#!/usr/bin/env python3
"""Continuous measurement script for SLAM pose estimation error vs Gazebo ground truth during navigation.

    python3 tools/map_quality.py [label]
"""
import math
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LABEL = sys.argv[1] if len(sys.argv) > 1 else ''
WAYPOINTS = [(4.3, 0.6), (0.0, 0.0), (4.3, 2.6), (0.0, 0.0)]


def gt():
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
    return x, y


class Q(Node):
    def __init__(self):
        super().__init__('map_quality')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def xy(self, parent, child):
        t = self.buf.lookup_transform(parent, child, Time())
        return t.transform.translation.x, t.transform.translation.y

    def goto(self, gx, gy, timeout=180.0, sampler=None):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(gx), float(gy)
        g.pose.pose.orientation.w = 1.0
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return 0
        res = gh.get_result_async()
        t0 = time.time()
        last_s = t0
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if sampler and time.time() - last_s >= 0.5:
                sampler()
                last_s = time.time()
        return res.result().status if res.done() else 0


rclpy.init()
n = Q()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 4:
    rclpy.spin_once(n, timeout_sec=0.1)

errs = []
corr = []
prev_mo = [None]


def sample():
    try:
        sx, sy = n.xy('map', 'base_footprint')
        g = gt()
        if g:
            errs.append(math.hypot(sx - g[0], sy - g[1]))
        mo = n.xy('map', 'odom')
        if prev_mo[0] is not None:
            corr.append(math.hypot(mo[0] - prev_mo[0][0], mo[1] - prev_mo[0][1]))
        prev_mo[0] = mo
    except Exception:
        pass


print('===== Map Quality [%s] =====' % LABEL)
sts = []
for wx, wy in WAYPOINTS:
    sts.append(n.goto(wx, wy, sampler=sample))

print('Waypoint status results %s (4=SUCCEEDED), samples: %d' % (sts, len(errs)))
if errs:
    errs.sort()
    print('SLAM pose error vs ground truth:')
    print('   median %.3f m | 90%% %.3f m | max %.3f m'
          % (errs[len(errs) // 2], errs[int(len(errs) * 0.9)], errs[-1]))
if corr:
    big = [c for c in corr if c > 0.05]
    print('map->odom correction (map fluctuation):')
    print('   avg/s %.4f m | max %.3f m | >5cm corrections: %d / %d'
          % (sum(corr) / len(corr), max(corr), len(big), len(corr)))
rclpy.shutdown()
sys.exit(0)
