#!/usr/bin/env python3
"""Global path direction fluctuation and controller angular velocity sign flip frequency script.

    python3 tools/path_stability.py [duration_s]
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

LOOK = 0.5          # Lookahead distance along path [m]
FLIP = 0.35         # Angular velocity threshold for sign flip [rad/s]


class PS(Node):

    def __init__(self):
        super().__init__('path_stability')
        self.xy = None
        self.last_xy = None
        self.dist = 0.0

        self.prev_head = None
        self.turns = []          # Heading angle change between consecutive plans [deg]
        self.plan_t = []
        self.wz = []             # (t, wz)
        self.flips = 0
        self.last_sign = 0

        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)
        self.xy = (p.x, p.y)

    def _plan(self, m):
        if self.xy is None or len(m.poses) < 2:
            return
        acc, prev = 0.0, None
        target = None
        for ps in m.poses:
            p = (ps.pose.position.x, ps.pose.position.y)
            if prev is not None:
                acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
                if acc >= LOOK:
                    target = p
                    break
            prev = p
        if target is None:
            return

        head = math.atan2(target[1] - self.xy[1], target[0] - self.xy[0])
        now = time.time()
        if self.prev_head is not None:
            d = math.atan2(math.sin(head - self.prev_head), math.cos(head - self.prev_head))
            self.turns.append(abs(math.degrees(d)))
            self.plan_t.append(now)
        self.prev_head = head

    def _cmd(self, m):
        w = m.twist.angular.z
        t = time.time()
        self.wz.append((t, w))
        s = 1 if w > FLIP else (-1 if w < -FLIP else 0)
        if s != 0:
            if self.last_sign != 0 and s != self.last_sign:
                self.flips += 1
            self.last_sign = s


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    n = PS()
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    el = time.time() - t0

    print()
    print('=' * 66)
    print('%.0f s observation / actual distance %.1f m (%.3f m/s)' % (el, n.dist, n.dist / el))
    print('=' * 66)

    if n.turns:
        t = sorted(n.turns)
        big = sum(1 for v in t if v > 60)
        rev = sum(1 for v in t if v > 120)
        rate = len(n.plan_t) / el
        print('(a) Global path initial heading (%.2f m ahead) fluctuation -- replan count %d, %.2f Hz'
              % (LOOK, len(n.plan_t), rate))
        print('    median %.1f deg / 90%% %.1f deg / max %.1f deg'
              % (t[len(t) // 2], t[int(len(t) * 0.9)], t[-1]))
        print('    >60 deg %d (%.0f%%), >120 deg (reversal) %d (%.0f%%)'
              % (big, 100 * big / len(t), rev, 100 * rev / len(t)))
    else:
        print('(a) Insufficient /plan messages received.')

    print()
    if n.wz:
        turning = [w for _, w in n.wz if abs(w) > FLIP]
        print('(b) Angular velocity command oscillation -- samples: %d' % len(n.wz))
        print('    |wz| > %.2f intervals: %d (%.0f%%), sign flips: %d (%.2f flips/s)'
              % (FLIP, len(turning), 100 * len(turning) / len(n.wz),
                 n.flips, n.flips / el))
    print()
    print('Interpretation: Large/frequent heading changes -> (a) Path flipping.')
    print('                Stable path but frequent sign flips -> (b) Controller oscillation.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
