#!/usr/bin/env python3
"""Node CPU and memory usage sampling script during fixed scenario execution.

    python3 tools/measure_load.py <label>

Scenario: c070 corridor traversal and return. Performs differential sampling using `/proc/<pid>/stat`.
"""
import os
import subprocess
import sys
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'baseline'
WAYPOINTS = [(4.3, 0.6), (0.0, 0.0)]
INTERVAL = 0.5

# Nodes active on target robot hardware
ROBOT_NODES = ['rtabmap', 'rgbd_odometry', 'controller_serv', 'planner_server',
               'bt_navigator', 'behavior_server', 'smoother_server',
               'waypoint_follow', 'velocity_smooth', 'lifecycle_mana',
               'robot_state_pub', 'twist_mux', 'imu_filter_madg',
               'component_conta', 'vodom_tf_relay', 'ekf_node',
               'frontier_explor']
# Simulation/visualization-only nodes
SIM_NODES = ['gz', 'parameter_bridg', 'rviz2', 'ruby']
WATCH = set(ROBOT_NODES) | set(SIM_NODES)

HZ = os.sysconf('SC_CLK_TCK')
PAGE_KB = os.sysconf('SC_PAGE_SIZE') / 1024.0

samples = []          # [{comm: (cpu_percent, rss_mb)}]
stop = threading.Event()


def scan():
    """{pid: (comm, ticks, rss_kb)} - Monitored processes only."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % pid, 'rb') as f:
                raw = f.read().decode('utf-8', 'replace')
        except OSError:
            continue
        i, j = raw.find('('), raw.rfind(')')
        if i < 0 or j < 0:
            continue
        comm = raw[i + 1:j]
        if not any(w in comm for w in WATCH):
            continue
        f = raw[j + 2:].split()
        try:
            ticks = int(f[11]) + int(f[12])
            rss_kb = int(f[21]) * PAGE_KB
        except (IndexError, ValueError):
            continue
        out[int(pid)] = (comm, ticks, rss_kb)
    return out


def sampler():
    prev, prev_t = scan(), time.time()
    while not stop.is_set():
        time.sleep(INTERVAL)
        cur, cur_t = scan(), time.time()
        dt = cur_t - prev_t
        if dt <= 0:
            continue
        row = {}
        for pid, (comm, ticks, rss) in cur.items():
            if pid not in prev or prev[pid][0] != comm:
                continue
            cpu = (ticks - prev[pid][1]) / HZ / dt * 100.0
            c, r = row.get(comm, (0.0, 0.0))
            row[comm] = (c + cpu, r + rss)
        if row:
            samples.append(row)
        prev, prev_t = cur, cur_t


class M(Node):
    def __init__(self):
        super().__init__('measure_load')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goto(self, gx, gy, timeout=180.0):
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
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return res.result().status if res.done() else 0


rclpy.init()
n = M()
n.ac.wait_for_server(timeout_sec=60.0)
t = time.time()
while time.time() - t < 3:
    rclpy.spin_once(n, timeout_sec=0.1)

th = threading.Thread(target=sampler, daemon=True)
th.start()
t0 = time.time()
results = [n.goto(x, y) for x, y in WAYPOINTS]
elapsed = time.time() - t0
stop.set()
th.join(timeout=3)

names = sorted({k for s in samples for k in s})
print('===== Load Measurement [%s] =====' % LABEL)
print('Scenario duration: %.0f s, samples: %d, goal status: %s (4=SUCCEEDED)'
      % (elapsed, len(samples), results))
print('%-18s %9s %9s %10s' % ('Node', 'Avg CPU%', 'Max CPU%', 'Max RSS MB'))


def stat(name):
    vals = [s[name][0] for s in samples if name in s]
    rss = [s[name][1] for s in samples if name in s]
    if not vals:
        return 0.0, 0.0, 0.0
    return sum(vals) / len(vals), max(vals), max(rss) / 1024.0


tot_cpu = tot_rss = 0.0
for nm in names:
    if nm not in ROBOT_NODES:
        continue
    st = stat(nm)
    print('%-18s %9.1f %9.1f %10.0f' % (nm, st[0], st[1], st[2]))
    tot_cpu += st[0]
    tot_rss += st[2]
print('%-18s %9.1f %9s %10.0f' % ('--- Total (Dev PC) ---', tot_cpu, '', tot_rss))
print('   = %.2f cores (on dev host)' % (tot_cpu / 100.0))
print()
for nm in names:
    if nm not in SIM_NODES:
        continue
    st = stat(nm)
    print('(Sim only) %-12s %8.1f %9.1f %10.0f' % (nm, st[0], st[1], st[2]))
rclpy.shutdown()
sys.exit(0)
