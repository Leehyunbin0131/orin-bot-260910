#!/usr/bin/env python3
"""In-corridor in-place rotation geometry verification script.

    python3 tools/turnaround_test.py [corridor_width_m...]
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

WORLD = 'room'
WALL_X = 3.2            # Center X of corridor wall bank
_ALL = [('0.90m', 2.6, 0.90), ('0.70m', 0.6, 0.70),
        ('0.60m', -3.0, 0.60), ('0.55m', -1.6, 0.55)]
CASES = [c for c in _ALL if len(sys.argv) < 2 or c[0] in sys.argv[1:]]

WZ = 0.5                # Rotation angular velocity [rad/s]
SPIN_T = 2 * math.pi / WZ * 1.35     # 360 degrees + margin


def set_pose(x, y, yaw):
    req = ('name: "orinbot", position: {z: 0.06}, '
           f'orientation: {{ z: {math.sin(yaw/2):.6f}, w: {math.cos(yaw/2):.6f} }}')
    cmd = f'gz service -s /world/{WORLD}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req \'{req}\''
    subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req',
         f'name: "orinbot", position {{ x: {x:.4f} y: {y:.4f} z: 0.06 }} orientation {{ z: {math.sin(yaw/2):.6f} w: {math.cos(yaw/2):.6f} }}'],
        capture_output=True, text=True, timeout=15)


def ground_truth():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    b = out[i:i + 300]
    try:
        x = float(b[b.find('x: ') + 3:b.find('\n', b.find('x: '))])
        j = b.find('y: ', b.find('x: '))
        y = float(b[j + 3:b.find('\n', j)])
        j = b.find('z: ', j + 5)
        j = b.find('z: ', j + 5)
        qz = float(b[b.find('z: ', j) + 3:b.find('\n', b.find('z: ', j))])
        qw = float(b[b.find('w: ', j) + 3:b.find('\n', b.find('w: ', j))])
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        return x, y, yaw
    except Exception:
        return None


class T(Node):
    def __init__(self):
        super().__init__('turnaround_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def spin_cmd(self, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.angular.z = float(wz)
        self.pub.publish(m)


rclpy.init()
n = T()
OFFSETS = [0.06, 0.10, 0.14, 0.20]


def try_spin(x, y):
    set_pose(x - 0.9, y, 0.0)
    time.sleep(2.5)
    t0 = time.time()
    while time.time() - t0 < 8.0:
        n.spin_cmd(0.20)
        rclpy.spin_once(n, timeout_sec=0.02)
        p = ground_truth()
        if p and p[0] > x:
            break
    n.spin_cmd(0.0)
    time.sleep(1.0)
    st = ground_truth()
    if not st:
        return 0.0, 99.0
    t0 = time.time()
    prev_y = st[2]
    total_yaw = 0.0
    maxd = 0.0
    while time.time() - t0 < SPIN_T:
        n.spin_cmd(WZ)
        rclpy.spin_once(n, timeout_sec=0.02)
        p = ground_truth()
        if p:
            dy = math.atan2(math.sin(p[2] - prev_y), math.cos(p[2] - prev_y))
            total_yaw += dy
            prev_y = p[2]
            maxd = max(maxd, math.hypot(p[0] - st[0], p[1] - st[1]))
    n.spin_cmd(0.0)
    time.sleep(1.0)
    return abs(math.degrees(total_yaw)), maxd


print('=== In-corridor in-place rotation test (Robot diagonal 0.566 m) ===')
header = '%-7s %9s' % ('Width', 'Margin')
for o in OFFSETS:
    header += ' %9s' % ('%.0fmm offset' % (o * 1000))
print(header + '   Tolerable offset')

for label, gy, wid in CASES:
    margin = (wid - 0.4 * math.sqrt(2)) / 2 * 1000
    row = '%-7s %8.0fmm' % (label, margin)
    worst_ok = -1.0
    for off in OFFSETS:
        y = gy + off
        deg, drift = try_spin(WALL_X, y)
        ok = deg >= 340 and drift < 0.40
        if ok and off > worst_ok:
            worst_ok = off
        if deg < 340:
            res = 'FAIL(%d deg)' % int(deg)
        elif drift >= 0.40:
            res = 'DRIFT(%.0fmm)' % (drift * 1000)
        else:
            res = 'PASS(%.0fmm)' % (drift * 1000)
        row += ' %9s' % res
    print(row + '   %s' % ('%.0f mm' % (worst_ok * 1000) if worst_ok >= 0 else 'None'))

n.spin_cmd(0.0)
rclpy.shutdown()
sys.exit(0)
