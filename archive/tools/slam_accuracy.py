#!/usr/bin/env python3
"""SLAM pose accuracy and return error verification script.

    python3 tools/slam_accuracy.py [label]
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LABEL = sys.argv[1] if len(sys.argv) > 1 else ''


def gt_pose():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 300]
    j = blk.find('x: ')
    x = float(blk[j + 3:blk.find('\n', j)])
    j = blk.find('y: ', j)
    y = float(blk[j + 3:blk.find('\n', j)])
    j = blk.find('z: ', j + 5)
    j = blk.find('z: ', j + 5)
    qx = float(blk[blk.find('x: ', j) + 3:blk.find('\n', blk.find('x: ', j))])
    qy = float(blk[blk.find('y: ', j) + 3:blk.find('\n', blk.find('y: ', j))])
    qz = float(blk[blk.find('z: ', j) + 3:blk.find('\n', blk.find('z: ', j))])
    qw = float(blk[blk.find('w: ', j) + 3:blk.find('\n', blk.find('w: ', j))])
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class A(Node):
    def __init__(self):
        super().__init__('slam_accuracy')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def slam_pose(self):
        t = self.buf.lookup_transform('map', 'base_footprint', Time())
        p, q = t.transform.translation, t.transform.rotation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def cmd(self, vx, wz, dur):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        t0 = time.time()
        while time.time() - t0 < dur:
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)


def ang_err(a, b):
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


rclpy.init()
n = A()
t = time.time()
while time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

print('===== SLAM Accuracy [%s] =====' % LABEL)

s0, g0 = n.slam_pose(), gt_pose()
print('Start  SLAM (%.3f, %.3f, %.2f deg) | Truth (%.3f, %.3f, %.2f deg)'
      % (s0[0], s0[1], math.degrees(s0[2]), g0[0], g0[1], math.degrees(g0[2])))

print('\n[1] In-place 360 degree rotation')
n.cmd(0.0, 0.5, 2 * math.pi / 0.5)
time.sleep(1.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
s1, g1 = n.slam_pose(), gt_pose()
print('    SLAM (%.3f, %.3f, %.2f deg) | Truth (%.3f, %.3f, %.2f deg)'
      % (s1[0], s1[1], math.degrees(s1[2]), g1[0], g1[1], math.degrees(g1[2])))
print('    >>> Position Error: %.3f m, Yaw Error: %.2f deg'
      % (math.hypot(s1[0] - g1[0], s1[1] - g1[1]), ang_err(s1[2], g1[2])))

print('\n[2] Forward 1.5 m -> 180 deg turn -> Forward 1.5 m (Return near origin)')
n.cmd(0.25, 0.0, 6.0)
n.cmd(0.0, 0.5, math.pi / 0.5)
n.cmd(0.25, 0.0, 6.0)
time.sleep(1.0)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
s2, g2 = n.slam_pose(), gt_pose()
print('    SLAM (%.3f, %.3f, %.2f deg) | Truth (%.3f, %.3f, %.2f deg)'
      % (s2[0], s2[1], math.degrees(s2[2]), g2[0], g2[1], math.degrees(g2[2])))
print('    >>> Position Error: %.3f m, Yaw Error: %.2f deg'
      % (math.hypot(s2[0] - g2[0], s2[1] - g2[1]), ang_err(s2[2], g2[2])))
rclpy.shutdown()
sys.exit(0)
