#!/usr/bin/env python3
"""Dock marker vision detection distance range, error, and blind spot measurement script.

    python3 tools/dock_range.py
"""

import math
import subprocess
import sys
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

DOCK_X = 1.0
MARKER_Y = -3.898 + 0.008 / 2.0     # Marker board front surface = -3.894
MARKER_Z = 0.31
DOCKED_Y = -3.60                    # Robot center at final docked pose
ROBOT_YAW = -1.5708                 # Facing towards dock

# Distances from robot center to marker surface [m]
OFFSETS = [2.4, 2.2, 2.0, 1.6, 1.3, 1.0, 0.8, 0.6, 0.50, 0.46, 0.44,
           0.42, 0.40, 0.38, 0.36, 0.34, 0.32, 0.30, 0.28,
           MARKER_Y * -1 + DOCKED_Y]


def teleport(y):
    subprocess.run(
        ['gz', 'service', '-s', '/world/room/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req',
         f'name: "orinbot", position {{ x: {DOCK_X:.3f} y: {y:.3f} z: 0.15 }} '
         f'orientation {{ z: -0.7071068 w: 0.7071068 }}'],
        capture_output=True, timeout=15)


def gt_y():
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/room/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=20).stdout
    i = out.find('name: "orinbot"')
    if i < 0:
        return None
    blk = out[i:i + 300]
    j = blk.find('y: ')
    if j < 0:
        return None
    return float(blk[j + 3:blk.find('\n', j)])


class Probe(Node):

    def __init__(self):
        super().__init__('dock_range')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.samples = []
        self.create_subscription(PoseStamped, 'detected_dock_pose', self._cb, 10)

    def _cb(self, msg):
        try:
            p = self.buf.transform(msg, 'base_footprint', timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return
        pos = p.pose.position
        o = p.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.samples.append((pos.x, pos.y, pos.z, yaw, o))

    def settle(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collect(self, seconds):
        self.samples = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.samples)


def dock_yaw_from(o):
    x, y, z, w = o.x, o.y, o.z, o.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    rclpy.init()
    n = Probe()
    print('Dist[m]  Rate     MeasDist   Error[mm] Lateral[mm] DockYaw[deg]')
    print('-------  ------  ---------  --------  --------  -----------')
    last_ok = None
    first_blind = None
    rows = []
    for d in OFFSETS:
        y = MARKER_Y + d
        if y < DOCKED_Y - 1e-6:
            continue
        teleport(y)
        n.settle(2.0)
        actual = gt_y()
        if actual is not None:
            d = actual - MARKER_Y
        s = n.collect(2.0)
        rate = min(1.0, len(s) / 30.0)
        if not s:
            print('%7.2f  %5.0f%%   (No detection)' % (d, 0))
            if last_ok is not None and first_blind is None:
                first_blind = d
            continue
        mx = sum(v[0] for v in s) / len(s)
        my = sum(v[1] for v in s) / len(s)
        yaws = [dock_yaw_from(v[4]) for v in s]
        myaw = math.atan2(sum(math.sin(v) for v in yaws) / len(yaws),
                          sum(math.cos(v) for v in yaws) / len(yaws))
        print('%7.2f  %5.0f%%  %9.4f  %+8.1f  %+8.1f  %+11.2f'
              % (d, rate * 100, mx, (mx - d) * 1000, my * 1000,
                 math.degrees(myaw)))
        rows.append((d, mx, my, myaw))
        last_ok = d

    print()
    if rows:
        near = min(rows, key=lambda r: r[0])
        print('Closest detection: %.2f m (measured %.4f, error %+.1f mm, yaw %+.2f deg)'
              % (near[0], near[1], (near[1] - near[0]) * 1000,
                 math.degrees(near[3])))
        far = max(rows, key=lambda r: r[0])
        print('Farthest detection: %.2f m' % far[0])
        target = DOCKED_Y - MARKER_Y
        bias = near[1] - near[0]
        print()
        print('Recommended external_detection_translation_x: %+.4f' % -(target + bias))
        print('  = -(docking target distance %.3f + close range bias %+.3f)' % (target, bias))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
