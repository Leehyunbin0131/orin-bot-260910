#!/usr/bin/env python3
"""Docking repetition success rate and alignment error verification script.

    python3 tools/dock_test.py [iterations]
"""

import math
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient
from rclpy.node import Node

DOCK_ID = 'home_dock'
DOCK_POSE = (1.0, -3.60, -1.5708)
START_POSE = (1.0, -2.0, -1.5708)
WARMUP = [(1.0, -1.2), (1.0, -2.0)]

TOL_LON = 0.048
TOL_LAT = 0.034
TOL_YAW = math.radians(5.0)


def gt_pose():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 300]
    try:
        j = blk.find('x: ')
        x = float(blk[j + 3:blk.find('\n', j)])
        j = blk.find('y: ', j)
        y = float(blk[j + 3:blk.find('\n', j)])
        j = blk.find('z: ', j + 5)
        j = blk.find('z: ', j + 5)
        qz = float(blk[j + 3:blk.find('\n', j)])
        j = blk.find('w: ', j)
        qw = float(blk[j + 3:blk.find('\n', j)])
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    except Exception:
        return None
    return x, y, yaw


class Tester(Node):

    def __init__(self):
        super().__init__('dock_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.nav_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dock_ac = ActionClient(self, DockRobot, 'dock_robot')
        self.undock_ac = ActionClient(self, UndockRobot, 'undock_robot')
        self.last_det = None
        self.states = []
        self.create_subscription(
            PoseStamped, '/detected_dock_pose', self._det, 10)

    def _det(self, _m):
        self.last_det = time.time()

    def _fb(self, fb_msg):
        st = fb_msg.feedback.state
        now = time.time()
        if not self.states or self.states[-1][0] != st:
            self.states.append((st, now))

    def _run(self, client, goal, timeout, feedback=None):
        if not client.wait_for_server(timeout_sec=30.0):
            return None, 'No action server'
        fut = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, 'Goal rejected'
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not res.done():
            gh.cancel_goal_async()
            return None, 'Timeout'
        return res.result(), None

    def goto(self, x, y, yaw=None, timeout=180.0):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        if yaw is None:
            g.pose.pose.orientation.w = 1.0
        else:
            g.pose.pose.orientation.z = math.sin(yaw / 2.0)
            g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        r, err = self._run(self.nav_ac, g, timeout)
        return r is not None and r.status == GoalStatus.STATUS_SUCCEEDED

    def do_dock(self, timeout=300.0):
        self.last_det = None
        self.states = []
        g = DockRobot.Goal()
        g.use_dock_id = True
        g.dock_id = DOCK_ID
        g.navigate_to_staging_pose = True
        t0 = time.time()
        r, err = self._run(self.dock_ac, g, timeout, feedback=self._fb)
        dur = time.time() - t0
        blind = (time.time() - self.last_det) if self.last_det else None
        return r, err, dur, blind

    def do_undock(self, timeout=60.0):
        g = UndockRobot.Goal()
        r, err = self._run(self.undock_ac, g, timeout)
        return r, err


STATE_NAME = {0: '-', 1: 'StagingNav', 2: 'Perceive', 3: 'Control', 4: 'ChargingWait', 5: 'Retry'}


def main():
    n_iter = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init()
    t = Tester()

    print('Preparing map -- executing initial warmup drive')
    for wx, wy in WARMUP:
        ok = t.goto(wx, wy)
        print('  (%.1f, %.1f) %s' % (wx, wy, 'Reached' if ok else 'Failed'))

    rows = []
    for i in range(n_iter):
        print('\n===== Iteration %d/%d =====' % (i + 1, n_iter))
        if i > 0:
            print('  Navigating to start pose...')
            t.goto(*START_POSE)

        r, err, dt, blind = t.do_dock()
        if err or r is None:
            print('  Docking failed: %s' % err)
            rows.append((False, None, None, None, dt, blind, err))
            continue
        ok = r.result.success
        code = r.result.error_code
        seq = ' -> '.join('%s' % STATE_NAME.get(s, s) for s, _ in t.states)
        print('  Result: %s (error_code=%d, retries=%d, %.1fs)'
              % ('SUCCESS' if ok else 'FAILED', code, r.result.num_retries, dt))
        print('  Stages: %s' % seq)
        if blind is not None:
            print('  No-detection duration: %.2f s' % blind)

        g = gt_pose()
        if g and ok:
            dx, dy = g[0] - DOCK_POSE[0], g[1] - DOCK_POSE[1]
            c, s = math.cos(DOCK_POSE[2]), math.sin(DOCK_POSE[2])
            lon = dx * c + dy * s
            lat = -dx * s + dy * c
            dyaw = math.atan2(math.sin(g[2] - DOCK_POSE[2]),
                              math.cos(g[2] - DOCK_POSE[2]))
            print('  Physical pose error -- Lon %+.1f mm / Lat %+.1f mm / Yaw %+.2f deg'
                  % (lon * 1000, lat * 1000, math.degrees(dyaw)))
            verdict = []
            verdict.append('Lon OK' if abs(lon) <= TOL_LON else 'Lon EXCEEDED')
            verdict.append('Lat OK' if abs(lat) <= TOL_LAT else 'Lat EXCEEDED')
            verdict.append('Yaw OK' if abs(dyaw) <= TOL_YAW else 'Yaw EXCEEDED')
            print('  Verdict: %s' % ', '.join(verdict))
            rows.append((ok, lon, lat, dyaw, dt, blind, None))
        else:
            rows.append((ok, None, None, None, dt, blind, None))

        print('  Undocking...')
        ur, uerr = t.do_undock()
        print('  Undocking %s' % ('SUCCESS' if (ur and ur.result.success) else
                             ('FAILED: %s' % uerr)))

    print('\n===== Summary =====')
    good = [r for r in rows if r[0]]
    print('Success: %d / %d' % (len(good), len(rows)))
    if good:
        def stat(idx, scale, unit):
            v = [abs(r[idx]) * scale for r in good if r[idx] is not None]
            if not v:
                return '-'
            return 'median %.1f %s, max %.1f %s' % (sorted(v)[len(v) // 2], unit,
                                                    max(v), unit)
        print('Lon error: %s' % stat(1, 1000, 'mm'))
        print('Lat error: %s' % stat(2, 1000, 'mm'))
        print('Yaw error: %s' % stat(3, 180 / math.pi, 'deg'))
        dts = sorted(r[4] for r in good)
        print('Elapsed time: median %.1f s, max %.1f s' % (dts[len(dts) // 2], dts[-1]))
        bl = [r[5] for r in good if r[5] is not None]
        if bl:
            print('No-detection duration: max %.2f s' % max(bl))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
