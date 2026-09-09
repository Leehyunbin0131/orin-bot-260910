#!/usr/bin/env python3
"""도크에 붙은 채 부팅했을 때 그 자세를 도크 좌표로 등록합니다.

    /battery_state (충전 중)  -->  [ DockRegister ]  --> 도크 DB 파일
                                                     --> staged_dock 파라미터

스테이션을 옮기면 로봇을 한 번 밀어 넣고 재부팅하는 것으로 끝납니다.
좌표를 손으로 읽어 옮겨 적을 필요가 없습니다.

**저장하는 것은 "도크 앞"이 아니라 "도킹 완료 자세"입니다.** 진입점은
staged_dock 이 approach_distance 로 계산합니다. 앞으로 빼서 저장하면 코드가
또 빼기 때문에 진입점이 두 배로 멀어집니다.

**후진 도킹이면 yaw 에서 180도를 빼야 합니다.** dock_yaw 는 "접근할 때
바라보는 방향"인데 후진 도킹의 최종 자세는 도크를 등지고 있어 정확히
반대입니다. 부호를 틀리면 로봇이 도크 반대편으로 진입점을 잡고 마커를
한 장도 못 봐서, 증상이 "마커 검출 실패"로만 보입니다.
"""

import math
import os
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DockRegister(Node):

    def __init__(self):
        super().__init__('dock_register')
        p = self.declare_parameter

        p('dock_id', 'home_dock')
        p('dock_type', 'orinbot_dock')
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        # docking.yaml 의 staged_dock.reverse_dock 과 같아야 합니다.
        p('reverse_dock', True)
        # Nav2 dock_database 규격 파일. docking_mode:=smooth 로 바꿔도
        # 그대로 쓸 수 있습니다.
        p('database_path', os.path.expanduser('~/.ros/orinbot_docks.yaml'))

        # 부팅 시 자동 등록 여부. 끄면 ~/register 서비스로만 동작합니다.
        p('auto_register', True)
        # 충전 전류가 이 값을 넘으면 "도크에 붙어 있다"로 봅니다.
        # 접촉은 물리적 사실이므로 위치 추정이 아니라 전류로 판정합니다.
        p('charge_threshold', 0.5)
        # 자세가 이만큼 안에서 이 횟수만큼 연속으로 머물면 SLAM 이
        # 자리를 잡은 것으로 봅니다.
        p('settle_tolerance', 0.01)       # [m]
        p('settle_samples', 10)
        p('settle_timeout', 120.0)        # [s]
        # 갱신된 좌표를 밀어 넣을 노드들 (dock_x / dock_y / dock_yaw 파라미터)
        p('target_nodes', ['/staged_dock'])

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.dock_id = g('dock_id')
        self.dock_type = g('dock_type')
        self.map_frame = g('map_frame')
        self.base = g('base_frame')
        self.reverse = g('reverse_dock')
        self.db_path = os.path.expanduser(g('database_path'))
        self.auto = g('auto_register')
        self.charge_thr = g('charge_threshold')
        self.settle_tol = g('settle_tolerance')
        self.settle_n = int(g('settle_samples'))
        self.settle_timeout = g('settle_timeout')
        self.targets = [t for t in g('target_nodes') if t]

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.current = None
        self.create_subscription(BatteryState, 'battery_state',
                                 self._battery, 10)
        self.create_service(Trigger, '~/register', self._srv_register)

        self.done = False
        if self.auto:
            # 한 번만 돌면 되므로 타이머로 조건을 기다립니다.
            self.timer = self.create_timer(2.0, self._try_auto)

        self.get_logger().info(
            '도크 등록 대기 — 자동 %s / DB %s / 후진 %s'
            % ('켬' if self.auto else '끔', self.db_path, self.reverse))

    # ------------------------------------------------------------------
    def _battery(self, msg):
        self.current = msg.current

    def _srv_register(self, _req, res):
        ok, msg = self.register(force=True)
        res.success, res.message = ok, msg
        return res

    def _try_auto(self):
        if self.done:
            return
        if self.current is None:
            return
        if self.current < self.charge_thr:
            self.get_logger().info(
                '충전 중이 아니므로 자동 등록하지 않습니다 (전류 %+.2f A). '
                '수동 등록은 ~/register 서비스입니다.' % self.current,
                once=True)
            self.done = True
            self.timer.cancel()
            return
        ok, msg = self.register(force=False)
        if ok:
            self.done = True
            self.timer.cancel()
        else:
            self.get_logger().warning('등록 보류 — %s' % msg)

    # ------------------------------------------------------------------
    def _settled_pose(self):
        """SLAM 이 자리를 잡을 때까지 기다렸다가 map->base 자세를 냅니다."""
        hist = []
        t0 = time.time()
        while time.time() - t0 < self.settle_timeout:
            try:
                t = self.buf.lookup_transform(
                    self.map_frame, self.base, rclpy.time.Time()).transform
            except Exception:                              # noqa: BLE001
                time.sleep(0.3)
                continue
            hist.append((t.translation.x, t.translation.y, yaw_of(t.rotation)))
            if len(hist) > self.settle_n:
                hist.pop(0)
            if len(hist) == self.settle_n:
                xs = [h[0] for h in hist]
                ys = [h[1] for h in hist]
                if (max(xs) - min(xs) < self.settle_tol
                        and max(ys) - min(ys) < self.settle_tol):
                    return hist[-1]
            time.sleep(0.3)
        return None

    def register(self, force):
        if self.current is None and not force:
            return False, '/battery_state 를 아직 못 받았습니다'
        if not force and (self.current or 0.0) < self.charge_thr:
            return False, '충전 중이 아닙니다 (전류 %+.2f A)' % (self.current or 0.0)

        pose = self._settled_pose()
        if pose is None:
            return False, '%s->%s 자세가 안정되지 않았습니다' % (self.map_frame, self.base)
        x, y, yaw_docked = pose

        # 후진 도킹이면 최종 자세가 접근 방향의 정반대입니다.
        dock_yaw = wrap(yaw_docked - math.pi) if self.reverse else yaw_docked

        try:
            self._write_db(x, y, dock_yaw)
        except Exception as exc:                           # noqa: BLE001
            return False, 'DB 기록 실패: %s' % exc

        self.get_logger().info(
            '도크 등록 — 위치 (%.3f, %.3f) / 도킹 시 자세 %+.1f도 '
            '-> 접근 방향 %+.1f도%s'
            % (x, y, math.degrees(yaw_docked), math.degrees(dock_yaw),
               ' (후진이라 180도 반대)' if self.reverse else ''))
        self.get_logger().info('DB 기록 완료: %s' % self.db_path)
        self._push_params(x, y, dock_yaw)
        return True, '(%.3f, %.3f, %.4f)' % (x, y, dock_yaw)

    def _write_db(self, x, y, yaw):
        """Nav2 dock_database 규격으로 씁니다 (docks/type/frame/pose)."""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        tmp = self.db_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write('# dock_register.py 가 자동 생성했습니다. 직접 고치지 마세요.\n')
            f.write('# pose 는 [x, y, yaw] 이고 yaw 는 **접근 방향**입니다\n')
            f.write('# (후진 도킹이면 도킹 완료 자세는 여기서 180도 돌아간 상태).\n')
            f.write('docks:\n')
            f.write('  %s:\n' % self.dock_id)
            f.write('    type: %s\n' % self.dock_type)
            f.write('    frame: %s\n' % self.map_frame)
            f.write('    pose: [%.4f, %.4f, %.4f]\n' % (x, y, yaw))
        os.replace(tmp, self.db_path)      # 원자적 교체 — 반쯤 쓰인 파일이 남지 않음

    def _push_params(self, x, y, yaw):
        """도는 노드에 좌표를 바로 반영합니다 (재시작 없이)."""
        for node in self.targets:
            cli = self.create_client(SetParameters, '%s/set_parameters' % node)
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning('%s 없음 — 파라미터 반영 건너뜀' % node)
                continue
            req = SetParameters.Request()
            for name, val in (('dock_x', x), ('dock_y', y), ('dock_yaw', yaw)):
                req.parameters.append(Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE, double_value=val)))
            fut = cli.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                time.sleep(0.05)
            res = fut.result()
            ok = res is not None and all(r.successful for r in res.results)
            self.get_logger().info(
                '%s 파라미터 반영 %s' % (node, '완료' if ok else '실패'))


def main():
    rclpy.init()
    n = DockRegister()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
