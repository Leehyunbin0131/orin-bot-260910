#!/usr/bin/env python3
"""TF duplicate publication and frame jump detection script.

    python3 tools/jump_check.py [duration_s]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


class J(Node):
    def __init__(self):
        super().__init__('jump_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.pubs = {}
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

    def on_tf(self, msg):
        for t in msg.transforms:
            key = '%s -> %s' % (t.header.frame_id, t.child_frame_id)
            self.pubs[key] = self.pubs.get(key, 0) + 1

    def xyq(self, parent, child):
        t = self.buf.lookup_transform(parent, child, Time())
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


rclpy.init()
n = J()
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

print('=== Transforms published on /tf (4s count) ===')
for k, v in sorted(n.pubs.items(), key=lambda x: -x[1]):
    print('   %-34s %5d times  (%.0f Hz)' % (k, v, v / 4.0))

print('\n=== Jump observations over %.0f seconds ===' % DUR)
prev_mo = prev_ob = None
jumps_mo, jumps_ob = [], []
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        mo = n.xyq('map', 'odom')
        ob = n.xyq('odom', 'base_footprint')
    except Exception:
        continue
    if prev_mo is not None:
        d = math.hypot(mo[0] - prev_mo[0], mo[1] - prev_mo[1])
        da = abs(math.atan2(math.sin(mo[2] - prev_mo[2]), math.cos(mo[2] - prev_mo[2])))
        if d > 0.02 or da > 0.02:
            jumps_mo.append((d, math.degrees(da)))
        d2 = math.hypot(ob[0] - prev_ob[0], ob[1] - prev_ob[1])
        if d2 > 0.05:
            jumps_ob.append(d2)
    prev_mo, prev_ob = mo, ob

print('map -> odom  (SLAM correction): jumps %d times' % len(jumps_mo))
if jumps_mo:
    print('   max %.3f m / %.2f deg, total displacement %.3f m'
          % (max(j[0] for j in jumps_mo), max(j[1] for j in jumps_mo),
             sum(j[0] for j in jumps_mo)))
print('odom -> base (Odometry): abnormal jumps %d times%s'
      % (len(jumps_ob),
         ('  max %.3f m' % max(jumps_ob)) if jumps_ob else '  (Normal)'))
rclpy.shutdown()
sys.exit(0)
