#!/usr/bin/env python3
"""Corridor traversal performance measurement script per width.

    python3 tools/corridor_test.py [width_m...]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# (Label, Opening center y, Width)
_ALL = [('0.90m', 2.6, 0.90), ('0.70m', 0.6, 0.70),
        ('0.60m', -3.0, 0.60), ('0.55m', -1.6, 0.55)]
CORRIDORS = [c for c in _ALL if len(sys.argv) < 2 or c[0] in sys.argv[1:]]
HOME = (0.0, 0.0)
TIMEOUT = 120.0


class C(Node):
    def __init__(self):
        super().__init__('corridor_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.signs = []
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def on_cmd(self, m):
        wz = m.twist.angular.z
        if abs(wz) > 0.05:
            self.signs.append(1 if wz > 0 else -1)

    def pose(self):
        t = self.buf.lookup_transform('map', 'base_footprint', Time())
        return t.transform.translation.x, t.transform.translation.y

    def goto(self, gx, gy, timeout=TIMEOUT):
        self.signs = []
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(gx), float(gy)
        g.pose.pose.orientation.w = 1.0
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, 0.0, 0
        res = gh.get_result_async()
        t0 = time.time()
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        st = res.result().status if res.done() else 0
        flips = sum(1 for a, b in zip(self.signs, self.signs[1:]) if a != b)
        return st, time.time() - t0, flips


rclpy.init()
n = C()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

print('Robot width 0.40 m. Corridor depth 0.60 m.')
print('%-8s %-7s %-9s %-7s %-6s %s' % ('Width', 'Margin', 'Result', 'Time', 'Flips', 'FinalPose'))
for label, gy, wdt in CORRIDORS:
    st, dt, fl = n.goto(4.3, gy)
    x, y = n.pose()
    ok = x > 3.6
    print('%-8s %-7s %-9s %5.0fs %5d  (%.2f, %.2f)'
          % (label, '%.3f m' % ((wdt - 0.40) / 2),
             'PASSED' if ok else ('FAILED(status=%s)' % st), dt, fl, x, y))
    st2, dt2, fl2 = n.goto(*HOME)
    x2, y2 = n.pose()
    if math.hypot(x2 - HOME[0], y2 - HOME[1]) > 0.5:
        print('         (Return home failed: %.2f, %.2f)' % (x2, y2))
rclpy.shutdown()
sys.exit(0)
