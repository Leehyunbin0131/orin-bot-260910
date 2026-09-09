#!/usr/bin/env python3
"""Robot stall and abnormal stop cause attribution script.

    python3 tools/stall_attribution.py [duration_s] [radius_m] [window_s]

Stall cause classification: No goal, BT recovery, No plan, Controller silent, Controller zero, Command blocked, Creeping.
"""

import math
import sys
import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

CMD_EPS_V, CMD_EPS_W = 0.005, 0.01
SILENT = 0.5
STALE_PLAN = 2.0

RECOVERY_NODES = ('BackUp', 'Spin', 'Wait', 'ClearEntireCostmap',
                  'ClearLocalCostmap', 'ClearGlobalCostmap', 'RecoveryActions',
                  'ClearingActions')

CAUSES = [
    ('NO_GOAL', 'No goal (explorer node has not issued a goal)'),
    ('BT_RECOVERY', 'BT recovery active (BackUp/Spin/Wait/Clear costmap)'),
    ('NO_PLAN', 'No plan (planner failure or stale path)'),
    ('CTRL_SILENT', 'Controller silent (no velocity commands output)'),
    ('CTRL_ZERO', 'Controller zero output (failed to find valid trajectory)'),
    ('CMD_BLOCKED', 'Command blocked (smoother/twist_mux pipeline)'),
    ('CREEPING', 'Command active but robot confined (slipping/stuck)'),
]


class Attrib(Node):

    def __init__(self, radius, window):
        super().__init__('stall_attribution')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.R = radius
        self.W = window

        self.xy = None
        self.hist = deque()

        self.goal_active = False
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._nav_status, 10)

        self.bt_active = set()
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self._bt_log, 10)

        self.last_plan_t = None
        self.create_subscription(Path, '/plan', lambda _m: setattr(self, 'last_plan_t', time.time()), 10)

        self.last_cin_t = None
        self.last_cin_v = 0.0
        self.last_cin_w = 0.0
        self.create_subscription(TwistStamped, '/cmd_vel_nav', self._cin, 10)

        self.last_cout_t = None
        self.last_cout_v = 0.0
        self.last_cout_w = 0.0
        self.create_subscription(TwistStamped, '/cmd_vel', self._cout, 10)

        self.lc = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'lc', m), TL)

        self.tally = defaultdict(float)
        self.episodes = []
        self.cur = None
        self.free_time = 0.0

        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.xy = (p.x, p.y)

    def _nav_status(self, m):
        self.goal_active = any(s.status == GoalStatus.STATUS_EXECUTING for s in m.status_list)

    def _bt_log(self, m):
        for e in m.event_log:
            if any(k in e.node_name for k in RECOVERY_NODES):
                if e.previous_status != 'IDLE' and e.current_status == 'IDLE':
                    self.bt_active.discard(e.node_name)
                elif e.current_status == 'RUNNING':
                    self.bt_active.add(e.node_name)
                elif e.current_status in ('SUCCESS', 'FAILURE'):
                    self.bt_active.discard(e.node_name)

    def _cin(self, m):
        self.last_cin_t = time.time()
        self.last_cin_v = m.twist.linear.x
        self.last_cin_w = m.twist.angular.z

    def _cout(self, m):
        self.last_cout_t = time.time()
        self.last_cout_v = m.twist.linear.x
        self.last_cout_w = m.twist.angular.z

    def _free_frac(self):
        if self.lc is None or self.xy is None:
            return float('nan')
        info = self.lc.info
        res = info.resolution
        grid = np.asarray(self.lc.data, dtype=np.int16).reshape(info.height, info.width)
        cx = int((self.xy[0] - info.origin.position.x) / res)
        cy = int((self.xy[1] - info.origin.position.y) / res)
        r_cells = int(1.0 / res)
        y0, y1 = max(0, cy - r_cells), min(info.height, cy + r_cells + 1)
        x0, x1 = max(0, cx - r_cells), min(info.width, cx + r_cells + 1)
        if y0 >= y1 or x0 >= x1:
            return float('nan')
        sub = grid[y0:y1, x0:x1]
        valid = sub >= 0
        if not valid.any():
            return float('nan')
        return float((sub[valid] < 99).sum() / valid.sum())

    def _confined(self, now):
        while self.hist and now - self.hist[0][0] > self.W:
            self.hist.popleft()
        if not self.hist or now - self.hist[0][0] < self.W * 0.9:
            return None, False
        cx = sum(h[1] for h in self.hist) / len(self.hist)
        cy = sum(h[2] for h in self.hist) / len(self.hist)
        far = max(math.dist((cx, cy), (h[1], h[2])) for h in self.hist)
        return (cx, cy), far <= self.R

    def diagnose(self, now):
        if not self.goal_active:
            return 'NO_GOAL'
        if self.bt_active:
            return 'BT_RECOVERY'
        if self.last_plan_t is None or now - self.last_plan_t > STALE_PLAN:
            return 'NO_PLAN'
        if self.last_cin_t is None or now - self.last_cin_t > SILENT:
            return 'CTRL_SILENT'
        cin_moving = abs(self.last_cin_v) > CMD_EPS_V or abs(self.last_cin_w) > CMD_EPS_W
        if not cin_moving:
            return 'CTRL_ZERO'
        cout_moving = abs(self.last_cout_v) > CMD_EPS_V or abs(self.last_cout_w) > CMD_EPS_W
        if not cout_moving:
            return 'CMD_BLOCKED'
        return 'CREEPING'

    def tick(self, dt):
        if self.xy is None:
            return
        now = time.time()
        self.hist.append((now, self.xy[0], self.xy[1]))
        center, confined = self._confined(now)
        if not confined:
            self.free_time += dt
            if self.cur:
                self.cur['ff1'] = self._free_frac()
                self.episodes.append(self.cur)
                self.cur = None
            return

        cause = self.diagnose(now)
        self.tally[cause] += dt
        if self.cur is None:
            self.cur = {
                'center': center, 't0': now, 'dur': 0.0,
                'causes': defaultdict(float), 'nodes': set(),
                'ff0': self._free_frac(), 'ff1': self._free_frac()
            }
        self.cur['dur'] += dt
        self.cur['causes'][cause] += dt
        self.cur['nodes'].update(self.bt_active)

    def report(self):
        if self.cur:
            self.cur['ff1'] = self._free_frac()
            self.episodes.append(self.cur)
            self.cur = None

        stuck_t = sum(self.tally.values())
        total = stuck_t + self.free_time
        print()
        print('=' * 72)
        print('Confined threshold: recent %.0f s positions within radius %.2f m' % (self.W, self.R))
        print('Total %.0f s: Normal motion %.0f s (%.0f%%) / Confined %.0f s (%.0f%%)'
              % (total, self.free_time, 100 * self.free_time / max(total, 1e-9),
                 stuck_t, 100 * stuck_t / max(total, 1e-9)))
        print('=' * 72)
        if stuck_t <= 0:
            print('No confinement episodes detected.')
            return
        print('%-13s %8s %7s   %s' % ('Cause', 'Seconds', 'Stuck%', 'Description'))
        for key, desc in CAUSES:
            s = self.tally.get(key, 0.0)
            if s > 0:
                print('%-13s %8.1f %6.0f%%   %s'
                      % (key, s, 100 * s / stuck_t, desc))
        print()
        print('Top 10 longest confined regions')
        print('%8s  %-18s %-13s %-13s %s'
              % ('Duration', 'Center', 'Main Cause', 'Free Frac Start->End', 'BT Nodes'))
        for e in sorted(self.episodes, key=lambda x: -x['dur'])[:10]:
            if e['dur'] < 2.0:
                continue
            top = max(e['causes'].items(), key=lambda kv: kv[1])
            nodes = ','.join(sorted(n for n in e['nodes'] if n))[:28]
            print('%6.1f s  (%6.2f, %6.2f)   %-13s %.2f -> %.2f     %s'
                  % (e['dur'], e['center'][0], e['center'][1], top[0],
                     e['ff0'], e['ff1'], nodes or '-'))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    radius = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    window = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    rclpy.init()
    n = Attrib(radius, window)
    t0 = time.time()
    last = t0
    try:
        while time.time() - t0 < dur:
            time.sleep(0.1)
            now = time.time()
            n.tick(now - last)
            last = now
            rclpy.spin_once(n, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    n.report()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
