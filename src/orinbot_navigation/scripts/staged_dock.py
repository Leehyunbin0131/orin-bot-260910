#!/usr/bin/env python3
"""단계 분리 도킹/언도킹 서버.

    /dock_robot   (nav2_msgs/DockRobot)
    /undock_robot (nav2_msgs/UndockRobot)

액션 규격은 opennav_docking 과 같지만, 곡선으로 붙는 대신 정지 -> 측정 ->
보정 -> 정지 -> 재측정 을 반복해 **정지 상태의 인식 정확도를 그대로 최종
자세로 옮깁니다.** 횡오차는 차동구동이 옆으로 못 가므로 크랩 기동으로
없앱니다. 설계 근거는 docs/ros2-lessons.md 6장.
"""

import math
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped transform registration)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

# DockRobot.Feedback status values (matching opennav_docking spec)
FB_NAV, FB_PERCEIVE, FB_CONTROL, FB_RETRY = 1, 2, 3, 5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class StagedDock(Node):

    def __init__(self):
        super().__init__('staged_dock')
        p = self.declare_parameter
        self.cb = ReentrantCallbackGroup()

        # 도크 자세 — docking.yaml 의 home_dock.pose 와 일치해야 합니다
        p('dock_id', 'home_dock')
        p('dock_x', 1.0)
        p('dock_y', -3.67)
        p('dock_yaw', -1.5708)
        # 마커면에서 최종 도킹 자세까지 [m]
        p('dock_distance', 0.224)
        # 후진 도킹: 정렬은 도크를 마주 본 채로 하고, 회전점에서 180도 돌아
        # 뒤로 들어갑니다. 충전 내내 카메라가 벽이 아니라 방을 보게 하려는
        # 것입니다 — 벽 0.3 m 앞에서는 시각 오도메트리가 깨집니다.
        p('reverse_dock', True)
        # 180도 회전 지점까지의 거리 [m]. 제약은 섀시가 아니라 **캐스터**
        # 입니다 — 회전 반지름 0.198 m 에 구 반지름이 0.030 m 뿐이라
        # 높이 0.040 m 인 동판 턱에 걸립니다. docking.yaml 주석 참고.
        p('rotate_distance', 0.55)
        # 회전점 허용 오차 [m]. 이 안에 들면 더 보정하지 않습니다.
        p('rotate_tolerance', 0.02)

        # --- 후진 구간을 뒤쪽 라이다로 닫습니다 ---
        # 180도 회전 뒤에는 마커가 등 뒤라 보이지 않아, 후진이 오도메트리
        # 개루프가 됩니다. 회전 중 생긴 밀림까지 그대로 최종 세로 오차로
        # 남습니다. 라이다는 360도라 뒤쪽 빔이 이미 있고, 스캔 평면이 도크
        # 패널보다 높아 그 너머 벽을 직접 봅니다.
        p('use_rear_lidar', True)
        # 도킹 완료 시 뒤쪽 라이다가 읽어야 할 거리 [m].
        #   벽 안쪽면 -3.95 / 도킹 시 로봇 중심 -3.67 / 라이다는 중심에서
        #   전방 0.15 인데 회전 후엔 그 방향이 벽 반대쪽이므로
        #   (-3.67 + 0.15) - (-3.95) = 0.43
        # **도크나 로봇 기하를 바꾸면 이 값도 다시 계산해야 합니다.**
        p('rear_target', 0.43)
        # 정후방에서 이 각도 안의 빔만 씁니다 [rad]. 각 빔을 후진 축으로
        # 투영하므로 창이 넓어도 편향은 없지만, 좁게 두어 벽이 아닌 것이
        # 섞이는 것을 막습니다.
        p('rear_window', 0.0873)          # ±5도
        p('rear_tolerance', 0.005)
        p('rear_min_beams', 5)
        p('rear_timeout', 30.0)
        # 마커면에서 정렬 지점까지 [m]
        p('approach_distance', 0.65)

        # 정렬 판정 — 축별이 아니라 접촉 시점 예상 횡오차 하나로 합니다
        # (동판 허용 ±34 mm 의 일부를 예산으로 잡음). docking.yaml 참고.
        p('contact_lateral_budget', 0.006)
        # 각도 상한 [rad]. 정밀도가 아니라 측정 이상 감시용입니다.
        p('yaw_tolerance', 0.0175)        # 1.0도
        p('max_align_iters', 6)
        # 마커가 안 보일 때 한 번에 물러나는 거리 [m]
        p('search_backoff', 0.15)
        # 정렬에 필요한 도크와의 최소 여유 [m]
        p('min_standoff', 0.62)

        # 기동 파라미터
        p('crab_angle', 0.5236)           # 30도
        p('v_rotate', 0.35)               # [rad/s]
        p('v_forward', 0.08)              # [m/s]
        # 목표 직전 미세 접근 속도 [m/s] 와 정지 데드밴드 [m].
        # 둘이 함께 세로 오차의 하한을 정합니다.
        p('v_creep', 0.01)
        p('forward_tolerance', 0.0005)
        p('settle_time', 0.7)             # 정지 후 안정화 대기 [s]
        p('measure_samples', 12)
        p('measure_timeout', 5.0)
        p('undock_distance', 0.5)         # 이탈 주행 거리 [m]

        # 도킹 구간에 멈출 것들 (std_srvs/Empty). 실기에서 카메라 스트림을
        # 끊는 서비스가 생기면 여기에 이름만 추가하면 됩니다.
        p('slam_pause_services', ['/rtabmap/pause', '/vodom_tf_relay/pause'])
        p('slam_resume_services', ['/rtabmap/resume', '/vodom_tf_relay/resume'])

        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('map_frame', 'map')
        # Marker pose to dock pose rotation pitch [rad]
        p('marker_pitch', 1.5708)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.dock_id = g('dock_id')
        self.dock_pose = (g('dock_x'), g('dock_y'), g('dock_yaw'))
        self.D = g('dock_distance')
        self.reverse = g('reverse_dock')
        self.R = g('rotate_distance')
        self.rot_tol = g('rotate_tolerance')
        self.use_rear = g('use_rear_lidar')
        self.rear_target = g('rear_target')
        self.rear_window = g('rear_window')
        self.rear_tol = g('rear_tolerance')
        self.rear_min_beams = int(g('rear_min_beams'))
        self.rear_timeout = g('rear_timeout')
        # 정렬 뒤 직진해서 멈출 지점 — 후진 도킹이면 회전점입니다.
        self.stop_at = self.R if self.reverse else self.D
        self.A = g('approach_distance')
        self.yaw_tol = g('yaw_tolerance')
        self.budget = g('contact_lateral_budget')
        self.max_iters = int(g('max_align_iters'))
        self.search_back = g('search_backoff')
        self.min_standoff = g('min_standoff')
        self.crab = g('crab_angle')
        self.w_rot = g('v_rotate')
        self.v_fwd = g('v_forward')
        self.v_creep = g('v_creep')
        self.fwd_tol = g('forward_tolerance')
        self.settle = g('settle_time')
        self.n_samples = int(g('measure_samples'))
        self.meas_timeout = g('measure_timeout')
        self.undock_d = g('undock_distance')
        self.base = g('base_frame')
        self.odom = g('odom_frame')
        self.map_frame = g('map_frame')
        self.mpitch = g('marker_pitch')
        self.slam_pause = [x for x in g('slam_pause_services') if x]
        self.slam_resume = [x for x in g('slam_resume_services') if x]

        from tf2_ros import Buffer, TransformListener
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

        self.marker = None
        self.create_subscription(PoseStamped, 'detected_dock_pose',
                                 self._marker, 10, callback_group=self.cb)
        # 센서 QoS(BEST_EFFORT)로 받습니다 — 발행자가 RELIABLE 이어도 호환됩니다.
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self._scan,
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.cmd = self.create_publisher(TwistStamped, 'cmd_vel_dock', 10)

        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                   callback_group=self.cb)

        self.dock_as = ActionServer(
            self, DockRobot, 'dock_robot',
            execute_callback=self._do_dock,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.undock_as = ActionServer(
            self, UndockRobot, 'undock_robot',
            execute_callback=self._do_undock,
            goal_callback=lambda _gh: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cb)

        self.get_logger().info(
            'Staged dock server started: staging %.2f m, dock %.3f m, '
            'lateral budget %.0f mm (yaw limit %.1f deg)'
            % (self.A, self.D, self.budget * 1000, math.degrees(self.yaw_tol)))

    # ---------------- Inputs ----------------
    def _marker(self, msg):
        self.marker = msg

    def _scan(self, msg):
        self.scan = msg

    def _drive(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd.publish(m)

    def _stop(self):
        self._drive(0.0, 0.0)

    # ---------------- Measurement ----------------
    def measure(self):
        """Measure dock pose in stationary state -> (forward dist, lateral error, yaw error)."""
        self.marker = None
        t0 = time.time()
        samples = []
        while len(samples) < self.n_samples and time.time() - t0 < self.meas_timeout:
            time.sleep(0.05)
            if self.marker is None:
                continue
            b = _stamp0(self.marker)
            try:
                b = self.buf.transform(b, self.base, timeout=rclpy.duration.Duration(seconds=0.1))
            except Exception:
                continue
            psi = wrap(yaw_of(_rot_pitch(b.pose.orientation, self.mpitch)))
            mx, my = b.pose.position.x, b.pose.position.y
            lat = mx * math.sin(psi) - my * math.cos(psi)
            samples.append((mx, my, psi, lat))
        if len(samples) < 3:
            return None
        n = len(samples)
        mx = sum(s[0] for s in samples) / n
        my = sum(s[1] for s in samples) / n
        psi = sum(s[2] for s in samples) / n
        lat = sum(s[3] for s in samples) / n
        fwd = mx * math.cos(psi) + sum(s[1] for s in samples) / n * math.sin(psi)
        return fwd, lat, psi, n

    def predict(self, fwd, lat, psi):
        """Predict expected contact lateral error [m] when proceeding straight."""
        return lat - max(0.0, fwd - self.D) * math.sin(psi)

    # ---------------- Maneuver ----------------
    def rotate(self, delta):
        """In-place rotation using odometry feedback."""
        if abs(delta) < 1e-4:
            return True
        x0, y0, a0 = self._odom_pose()
        target = wrap(a0 + delta)
        t0 = time.time()
        while time.time() - t0 < 15.0:
            _, _, a = self._odom_pose()
            err = wrap(target - a)
            if abs(err) < 0.004:
                break
            w = max(0.05, min(self.w_rot, abs(err) * 1.5)) * (1 if err > 0 else -1)
            self._drive(0.0, w)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    def forward(self, dist):
        """Straight movement using odometry feedback."""
        if abs(dist) < 1e-4:
            return True
        x0, y0, _ = self._odom_pose()
        t0 = time.time()
        sign = 1 if dist > 0 else -1
        target = abs(dist)
        while time.time() - t0 < 30.0:
            x, y, _ = self._odom_pose()
            traveled = math.hypot(x - x0, y - y0)
            err = target - traveled
            if err <= self.fwd_tol:
                break
            # 최저 속도가 정지 분해능을 정합니다 — 제어 주기가 0.02 s 이므로
            # 한 틱에 v_creep * 0.02 만큼 더 갑니다. 세로 오차의 바닥이 여기라
            # 데드밴드(forward_tolerance)와 함께 정해야 합니다.
            v = max(self.v_creep, min(self.v_fwd, err * 1.0)) * sign
            self._drive(v, 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        return True

    # ------------------------------------------------------- 뒤쪽 라이다
    def rear_distance(self):
        """정후방 벽까지의 수직 거리 [m]. 못 재면 None.

        각 빔을 후진 축으로 투영(r*cos)해서 창 안의 빔이 비스듬히 맞아
        길게 나오는 편향을 없앱니다. 중앙값이라 이상치 몇 개는 무시됩니다.
        """
        s = self.scan
        if s is None:
            return None
        vals = []
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            off = abs(wrap(s.angle_min + i * s.angle_increment - math.pi))
            if off <= self.rear_window:
                vals.append(r * math.cos(off))
        if len(vals) < self.rear_min_beams:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def backward_to_rear(self, target, expect):
        """뒤쪽 라이다를 보며 목표 거리까지 후진합니다.

        expect 는 오도메트리로 예상한 후진량입니다. 라이다가 엉뚱한 것을
        보고 있을 때 도크로 밀고 들어가지 않도록, 이동량이 예상의 1.5배를
        넘으면 중단하는 안전장치로만 씁니다.
        """
        d0 = self.rear_distance()
        if d0 is None:
            return False
        limit = max(0.05, expect * 1.5)
        x0, y0, _ = self._odom_pose()
        t0 = time.time()
        while time.time() - t0 < self.rear_timeout:
            d = self.rear_distance()
            if d is None:
                self._stop()
                self.get_logger().warning('후진 중 뒤쪽 빔을 잃었습니다')
                return False
            err = d - target
            if err <= self.rear_tol:
                break
            x, y, _ = self._odom_pose()
            if math.hypot(x - x0, y - y0) > limit:
                self._stop()
                self.get_logger().error(
                    '후진량이 예상(%.3f m)의 1.5배를 넘었습니다 — 중단'
                    % expect)
                return False
            self._drive(-max(self.v_creep, min(self.v_fwd, err)), 0.0)
            time.sleep(0.02)
        self._stop()
        time.sleep(self.settle)
        d1 = self.rear_distance()
        self.get_logger().info(
            '후진 완료 — 뒤쪽 거리 %.3f -> %.3f m (목표 %.3f, 이동 %.3f m)'
            % (d0, d1 if d1 is not None else float('nan'), target,
               math.hypot(*[a - b for a, b in
                            zip(self._odom_pose()[:2], (x0, y0))])))
        return True

    def crab_move(self, shift):
        """Perform crab maneuver to shift laterally by shift amount."""
        s = abs(shift) / math.sin(self.crab)
        turn = -self.crab if shift > 0 else self.crab
        self.rotate(turn)
        self.forward(-s)
        self.rotate(-turn)

    # ---------------- Actions ----------------
    def _do_dock(self, handle):
        r = DockRobot.Result()
        fb = DockRobot.Feedback()

        def emit(state):
            fb.state = state
            handle.publish_feedback(fb)

        goal = handle.request
        # 도크 좌표는 매 시도마다 다시 읽습니다 — dock_register 가 런타임에
        # 갱신할 수 있으므로 __init__ 때 읽은 값을 그대로 쓰면 안 됩니다.
        self.dock_pose = (self.get_parameter('dock_x').value,
                          self.get_parameter('dock_y').value,
                          self.get_parameter('dock_yaw').value)
        self.get_logger().info(
            'Starting staged dock (dock_id=%s, dock (%.3f, %.3f, %+.4f))'
            % (goal.dock_id, *self.dock_pose))

        # Stage 1: Move to staging pose
        if goal.navigate_to_staging_pose:
            emit(FB_NAV)
            if not self._goto_entry():
                r.success, r.error_code = False, DockRobot.Result.FAILED_TO_STAGE
                r.error_msg = 'Failed to reach staging pose'
                handle.abort()
                return r

        # Nav2 의 각도를 믿으면 안 됩니다. 언도킹 직후에는 로봇이 이미 진입점
        # 0.1 m 안에 있어 위치 조건만으로 목표가 즉시 성공 처리되고, 도크를
        # 등진 채로 정렬이 시작돼 마커를 한 장도 못 봅니다. 여기서 지도
        # 기준 방위를 직접 확인하고 오도메트리로 돌려 세웁니다.
        self._face_dock()
        self._restore_standoff()

        # 여기서부터 지도 갱신과 시각 오도메트리 보정을 얼립니다.
        # 카메라가 벽을 0.3~0.9 m 앞에서 보며 제자리 회전과 크랩을 반복하는
        # 구간이라 시각 오도메트리가 깨지고, 그 오차가 포즈 그래프에 그대로
        # 들어가 지도를 망칩니다. 도킹은 odom 기준이고 이동량도 1 m 미만이라
        # 이 구간은 휠+IMU 로 충분합니다.
        self._freeze_slam(True)

        # Stages 2-4: Measure -> Align yaw -> Crab correction -> Remeasure
        emit(FB_PERCEIVE)
        aligned = None
        prev_pred = None

        for i in range(self.max_iters):
            if handle.is_cancel_requested:
                handle.canceled()
                return r
            m = self.measure()
            if m is None:
                # 지도 좌표로 "충분히 멀다"고 판단하면 안 됩니다 — SLAM 오차가
                # 수십 cm 면 실제로는 검출 절벽(0.55 m) 안쪽에 서 있을 수
                # 있습니다. 도크를 마주 본 상태이므로 뒤로 물러나면 마커가
                # 화각에 다시 들어옵니다.
                self.get_logger().warning(
                    '마커 미검출 (%d회) — %.2f m 물러나 다시 봅니다'
                    % (i + 1, self.search_back))
                emit(FB_RETRY)
                self.forward(-self.search_back)
                continue
            fwd, lat, psi, n = m
            pred = self.predict(fwd, lat, psi)
            self.get_logger().info(
                '  [%d] fwd %.3f m / lat %+.1f mm / yaw %+.2f deg '
                '-> predicted contact %+.1f mm (samples %d)'
                % (i + 1, fwd, lat * 1000, math.degrees(psi),
                   pred * 1000, n))
            if abs(pred) <= self.budget and abs(psi) <= self.yaw_tol:
                aligned = m
                break
            if abs(psi) > self.yaw_tol:
                self.rotate(psi)
                continue
            if prev_pred is not None and abs(pred) > abs(prev_pred) + 0.002:
                self.get_logger().error(
                    'Crab correction increased error (%.1f -> %.1f mm). Aborting alignment.'
                    % (prev_pred * 1000, pred * 1000))
                break
            prev_pred = pred
            self.crab_move(-pred)

        if aligned is None:
            r.success, r.error_code = False, DockRobot.Result.FAILED_TO_DETECT_DOCK
            r.error_msg = 'Dock detection failed during alignment'
            self._freeze_slam(False)     # 실패해도 반드시 되돌립니다
            handle.abort()
            return r

        # Stage 5: Straight entry
        emit(FB_CONTROL)
        fwd, lat, psi, _ = aligned

        if fwd < self.min_standoff:
            back = self.min_standoff - fwd
            self.get_logger().info(
                'Distance to dock %.3f m < min standoff, backing out %.3f m'
                % (fwd, back))
            self.forward(-back)
            m = self.measure()
            if m is not None:
                fwd, lat, psi, _ = m
                pred = self.predict(fwd, lat, psi)
                if abs(pred) > self.budget:
                    self.get_logger().info(
                        'Re-correcting lateral drift after backout (%+.1f mm)'
                        % (pred * 1000))
                    self.crab_move(-pred)
                    m = self.measure()
                    if m is not None:
                        fwd, lat, psi, _ = m

        run = max(0.0, fwd - self.stop_at)
        self.get_logger().info(
            'Alignment complete -- standoff %.3f m / lat %+.1f mm / yaw %+.2f deg '
            '-> predicted contact %+.1f mm (budget %.0f mm). '
            'Executing final %.3f m straight run'
            % (fwd, lat * 1000, math.degrees(psi),
               self.predict(fwd, lat, psi) * 1000, self.budget * 1000, run))
        if run > 0.001:
            self.forward(run)

        if self.reverse:
            # 회전점을 마커로 닫습니다. rotate_distance 는 검출 절벽(로봇
            # 중심 0.516 m) 위라 여기서도 마커가 보이고 거리 정확도가
            # ±1.4 mm 입니다. 회전 중 캐스터(스윕 반지름 0.198 m, 구 반지름
            # 0.030 m)가 높이 0.040 m 인 동판 턱을 넘지 못하므로 회전점이
            # 흔들리면 걸립니다.
            here = None
            for _ in range(3):
                m = self.measure()
                if m is None:
                    break
                here = m[0]
                adj = here - self.R
                if abs(adj) <= self.rot_tol:
                    break
                self.get_logger().info(
                    '회전점 보정 %+.0f mm (측정 %.3f m, 목표 %.3f ±%.0f mm)'
                    % (adj * 1000, here, self.R, self.rot_tol * 1000))
                self.forward(adj)
            if here is None:
                here = self.R
                self.get_logger().warning(
                    '회전점에서 마커 미검출 — 오도메트리 값 %.3f m 로 진행합니다'
                    % here)
            # **후진량은 고정값이 아니라 방금 잰 거리에서 뺍니다.** 회전 뒤에는
            # 마커가 뒤에 있어 보정할 수 없으므로, 여기서 남은 오차를 그대로
            # 후진량에 반영해야 도크까지 옮겨가지 않습니다.
            back = max(0.0, here - self.D)
            self.get_logger().info(
                '회전점 %.3f m — 180도 회전 후 %.3f m 후진합니다' % (here, back))
            self.rotate(math.pi)
            # 회전이 끝나야 뒤쪽 빔이 도크를 향합니다. 여기서부터는 마커를
            # 못 보므로, 오도메트리 대신 라이다로 목표 거리까지 닫습니다.
            # 이렇게 하면 회전 중 생긴 밀림도 함께 흡수됩니다.
            done = False
            if self.use_rear and back > 0.001:
                done = self.backward_to_rear(self.rear_target, back)
                if not done:
                    self.get_logger().warning(
                        '뒤쪽 라이다를 쓸 수 없어 오도메트리로 후진합니다')
            if not done and back > 0.001:
                self.forward(-back)

        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Docking complete')
        return r

    def _do_undock(self, handle):
        r = UndockRobot.Result()
        # 도킹 때 얼렸더라도 auto_dock 의 절전 해제가 먼저 풀어 놓았을 수 있어
        # 여기서 다시 겁니다. 후진 구간도 카메라가 벽을 보고 있습니다.
        self._freeze_slam(True)
        # 후진 도킹이면 로봇이 이미 벽을 등지고 있으므로 전진이 이탈입니다.
        out = self.undock_d if self.reverse else -self.undock_d
        self.get_logger().info(
            '언도킹 — %s %.2f m' % ('전진' if out > 0 else '후진', abs(out)))
        self.forward(out)
        self._stop()
        self._freeze_slam(False)
        r.success, r.error_code = True, 0
        handle.succeed()
        self.get_logger().info('Undocking complete')
        return r

    def _restore_standoff(self):
        """도크에 너무 붙어 있으면 물러섭니다.

        마커 3장이 다 보이는 한계가 0.55 m 부근이라, 앞선 시도가 실패해
        로봇이 도크 앞에 박힌 채 남으면 이후 모든 시도가 검출조차 못 합니다.
        """
        try:
            t = self.buf.lookup_transform(
                self.map_frame, self.base, rclpy.time.Time()).transform
        except Exception:                                  # noqa: BLE001
            return
        dx, dy, dyaw = self.dock_pose
        d = math.hypot(t.translation.x - dx, t.translation.y - dy) + self.D
        if d >= self.A:
            return
        back = self.A - d + 0.05
        self.get_logger().info(
            '도크에서 %.2f m 뿐이라 %.2f m 물러섭니다 (정렬에 %.2f m 필요)'
            % (d, back, self.A))
        self.forward(-back)

    def _face_dock(self):
        """도크를 마주 보도록 제자리에서 돌립니다 (지도 기준 방위 사용)."""
        want = self.dock_pose[2]
        for _ in range(3):
            try:
                t = self.buf.lookup_transform(
                    self.map_frame, self.base, rclpy.time.Time()).transform
            except Exception:                              # noqa: BLE001
                time.sleep(0.3)
                continue
            err = wrap(want - yaw_of(t.rotation))
            if abs(err) < math.radians(5.0):
                return True
            self.get_logger().info(
                '도크 방향으로 %+.1f도 회전' % math.degrees(err))
            self.rotate(err)
            return True
        self.get_logger().warning('map->base 조회 실패 — 방위 보정을 건너뜁니다')
        return False

    # ------------------------------------------------------------ 지도 동결
    def _freeze_slam(self, on):
        """도킹 구간 동안 포즈 그래프와 시각 오도메트리 보정을 멈춥니다."""
        for name in (self.slam_pause if on else self.slam_resume):
            cli = self.create_client(Empty, name)
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning('%s 없음 — 건너뜁니다' % name)
                continue
            fut = cli.call_async(Empty.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                time.sleep(0.02)
        self.get_logger().info('SLAM %s' % ('동결' if on else '재개'))

    def _goto_entry(self):
        """Navigate to approach staging pose."""
        dx, dy, dyaw = self.dock_pose
        back = self.A - self.D
        ex = dx - back * math.cos(dyaw)
        ey = dy - back * math.sin(dyaw)

        g = NavigateToPose.Goal()
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.header.frame_id = self.map_frame
        g.pose.pose.position.x = ex
        g.pose.pose.position.y = ey
        g.pose.pose.position.z = 0.0
        g.pose.pose.orientation.z = math.sin(dyaw / 2.0)
        g.pose.pose.orientation.w = math.cos(dyaw / 2.0)

        if not self.nav_ac.wait_for_server(timeout_sec=5.0):
            return False
        fut = self.nav_ac.send_goal_async(g)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        if not fut.done() or not fut.result().accepted:
            return False
        gh = fut.result()
        rfut = gh.get_result_async()
        t0 = time.time()
        while not rfut.done() and time.time() - t0 < 120.0:
            time.sleep(0.1)
        if not rfut.done():
            return False
        res = rfut.result()
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED

    def _odom_pose(self):
        try:
            tf = self.buf.lookup_transform(
                self.odom, self.base, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            p = tf.transform.translation
            r = tf.transform.rotation
            return p.x, p.y, yaw_of(r)
        except Exception:
            return 0.0, 0.0, 0.0


def _stamp0(msg):
    out = PoseStamped()
    out.header.frame_id = msg.header.frame_id
    out.pose = msg.pose
    return out


def _rot_pitch(q, pitch):
    """마커 자세에 y축 회전을 곱합니다: q (x) q_pitch.

    마커 광학 규약(z 가 마커 밖)에서 도크 규약(x 가 도크 정면)으로 옮기는
    변환이라, 축을 틀리면 yaw 에 90도가 통째로 섞여 들어갑니다.
    """
    ey, ew = math.sin(pitch / 2.0), math.cos(pitch / 2.0)
    return Quaternion(
        x=q.x * ew - q.z * ey,
        y=q.y * ew + q.w * ey,
        z=q.z * ew + q.x * ey,
        w=q.w * ew - q.y * ey)


def main():
    rclpy.init()
    n = StagedDock()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
