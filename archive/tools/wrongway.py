#!/usr/bin/env python3
"""Script to detect reverse rotation relative to path required direction.

    python3 tools/wrongway.py [duration_s]
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

LOOK = 0.5        # Lookahead distance along path [m]
THRESH = 20.0     # Angle threshold [deg]
WZ_MIN = 0.05     # Minimum angular velocity threshold [rad/s]


class WW(Node):

    def __init__(self):
        super().__init__('wrongway')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.xy = None
        self.yaw = None
        self.path = None
        self.samples = 0
        self.bad = 0
        self.events = []
        self.cur = None

        self.create_subscription(Odometry, '/odometry/filtered', self._od, 10)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'path', m), 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd, 20)

    def _od(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.xy = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y ** 2 + q.z ** 2))

    def _look_ahead(self):
        """Lookahead point LOOK m ahead along path."""
        if self.path is None or self.xy is None or len(self.path.poses) < 2:
            return None, False
        pts = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        acc, prev = 0.0, None
        d = [math.dist(self.xy, p) for p in pts]
        start = int(np.argmin(d)) if len(d) else 0
        tgt = None
        for i in range(start, len(pts)):
            if prev is not None:
                acc += math.dist(pts[i], prev)
                if acc >= LOOK:
                    tgt = pts[i]
                    break
            prev = pts[i]
        idx = [i for i, v in enumerate(d) if v < 0.6]
        folded = any(idx[k + 1] - idx[k] > 4 for k in range(len(idx) - 1)) if len(idx) > 1 else False
        return tgt, folded

    def _cmd(self, m):
        wz = m.twist.angular.z
        vx = m.twist.linear.x
        if self.yaw is None or self.path is None:
            return
        tgt, folded = self._look_ahead()
        if tgt is None:
            return
        head = math.atan2(tgt[1] - self.xy[1], tgt[0] - self.xy[0])
        err = math.degrees(math.atan2(math.sin(head - self.yaw), math.cos(head - self.yaw)))
        self.samples += 1
        is_bad = False
        if abs(err) >= THRESH and abs(wz) >= WZ_MIN:
            if (err > 0 and wz < 0) or (err < 0 and wz > 0):
                is_bad = True

        now = time.time()
        if is_bad:
            self.bad += 1
            if self.cur is None:
                self.cur = {'t': now, 'xy': self.xy, 'deg': err, 'wz': wz, 'vx': vx, 'folded': folded}
        else:
            if self.cur is not None:
                self.cur['dur'] = now - self.cur['t']
                self.events.append(self.cur)
                self.cur = None

    def report(self):
        if self.cur is not None:
            self.cur['dur'] = time.time() - self.cur['t']
            self.events.append(self.cur)
        print()
        print('=' * 70)
        print('Reverse rotation samples: %d / %d (%.1f%%), events: %d'
              % (self.bad, self.samples,
                 100 * self.bad / max(self.samples, 1), len(self.events)))
        print('=' * 70)
        ev = [e for e in self.events if e.get('dur', 0) > 0.3]
        if not ev:
            print('No reverse rotation events lasting >0.3 s detected.')
            return
        print('%7s  %-18s %9s %8s %8s  %s'
              % ('Dur[s]', 'Location', 'ReqAngle', 'CmdWz', 'CmdVx', 'PathFolded'))
        for e in sorted(ev, key=lambda x: -x['dur'])[:15]:
            print('%6.1fs  (%6.2f, %6.2f)  %7.0fdeg %8.2f %8.2f  %s'
                  % (e['dur'], e['xy'][0], e['xy'][1], e['deg'], e['wz'],
                     e['vx'], 'Yes' if e['folded'] else 'No'))
        tot = sum(e['dur'] for e in ev)
        fold = sum(e['dur'] for e in ev if e['folded'])
        print()
        print('Total reverse rotation time: %.1f s, during folded path: %.1f s (%.0f%%)'
              % (tot, fold, 100 * fold / max(tot, 1e-9)))


def main():
    import numpy as np  # Needed in _look_ahead
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    rclpy.init()
    n = WW()
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.report()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
