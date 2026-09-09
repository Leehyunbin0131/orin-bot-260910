#!/usr/bin/env python3
"""Script to calculate external_detection_rotation_* parameters for dock vision pose estimation.

    python3 tools/dock_calib.py
"""

import argparse
import itertools
import math
import sys
import time

import rclpy
import tf2_geometry_msgs  # noqa: F401  (register do_transform for PoseStamped)
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

HALF = math.pi / 2.0


def quat_mul(a, b):
    """Quaternion multiplication (x, y, z, w) order: a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Calib(Node):

    def __init__(self, frame):
        super().__init__('dock_calib')
        self.frame = frame
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.samples = []
        self.create_subscription(PoseStamped, 'detected_dock_pose', self._cb, 10)

    def _cb(self, msg):
        try:
            p = self.buf.transform(msg, self.frame, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return
        pos = p.pose.position
        o = p.pose.orientation
        self.samples.append(((pos.x, pos.y, pos.z), (o.x, o.y, o.z, o.w)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default='base_footprint',
                    help='Target frame to solve calibration for')
    ap.add_argument('--expect', type=float, default=0.0,
                    help='Expected dock yaw in target frame [rad] (0 for facing head-on)')
    ap.add_argument('--samples', type=int, default=30)
    a = ap.parse_args()

    rclpy.init()
    n = Calib(a.frame)
    t0 = time.time()
    while len(n.samples) < a.samples and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.samples:
        print('Failed to transform /detected_dock_pose to %s. Check detector and TF status.' % a.frame)
        rclpy.shutdown()
        return 1

    pos = [s[0] for s in n.samples]
    quats = [s[1] for s in n.samples]
    print('Collected %d samples in frame %s' % (len(n.samples), a.frame))
    print('Marker position (avg): x=%.4f y=%.4f z=%.4f'
          % tuple(sum(c[i] for c in pos) / len(pos) for i in range(3)))

    print('\nExpected dock yaw = %+.4f rad. Matching rotations:' % a.expect)
    hits = []
    for r, p, y in itertools.product(range(-1, 3), repeat=3):
        ext = rpy_to_quat(r * HALF, p * HALF, y * HALF)
        yaws = [quat_to_yaw(quat_mul(q, ext)) for q in quats]
        my = math.atan2(sum(math.sin(v) for v in yaws) / len(yaws),
                        sum(math.cos(v) for v in yaws) / len(yaws))
        spread = max(abs(math.atan2(math.sin(v - my), math.cos(v - my))) for v in yaws)
        err = abs(math.atan2(math.sin(my - a.expect), math.cos(my - a.expect)))
        if err < math.radians(8):
            hits.append((err, spread, r, p, y, my))
    if not hits:
        print('  None found -- check if robot is facing dock head-on and --expect value is correct.')
    for err, spread, r, p, y, my in sorted(hits):
        print('  roll=%+.4f pitch=%+.4f yaw=%+.4f  ->  dock yaw %+.4f '
              '(error %.2f deg, sample spread %.2f deg)'
              % (r * HALF, p * HALF, y * HALF, my,
                 math.degrees(err), math.degrees(spread)))

    print('\nRecommended docking.yaml configuration (minimal error candidate):')
    if hits:
        _e, _s, r, p, y, _m = sorted(hits)[0]
        print('      external_detection_rotation_roll: %.4f' % (r * HALF))
        print('      external_detection_rotation_pitch: %.4f' % (p * HALF))
        print('      external_detection_rotation_yaw: %.4f' % (y * HALF))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
