#!/usr/bin/env python3
"""Camera stream publish rate and timestamp synchronization status verification script.

    python3 tools/stream_rates.py [duration_s]
"""
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TOPICS = {
    'infra1': '/camera/infra1/image_rect_raw',
    'depth': '/camera/depth/image_rect_raw',
    'color': '/camera/color/image_raw',
}


class R(Node):
    def __init__(self):
        super().__init__('stream_rates')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.stamps = defaultdict(list)
        for name, topic in TOPICS.items():
            self.create_subscription(
                Image, topic,
                lambda m, n=name: self.stamps[n].append(
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                qos_profile_sensor_data)


rclpy.init()
n = R()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)

print('=== Stream rates over %.0f seconds ===' % DUR)
for name in TOPICS:
    s = n.stamps[name]
    if len(s) < 2:
        print('  %-8s received %d frames (none/inactive)' % (name, len(s)))
        continue
    span = s[-1] - s[0]
    print('  %-8s received %4d frames, sim time %.1f s -> %.1f Hz'
          % (name, len(s), span, (len(s) - 1) / span if span > 0 else 0))

a, b = n.stamps['infra1'], n.stamps['depth']
if a and b:
    matched = sum(1 for x in a if any(abs(x - y) < 1e-4 for y in b))
    print('\ninfra1 - depth timestamp exact match: %d / %d (%.0f%%)'
          % (matched, len(a), 100.0 * matched / len(a)))
    worst = 0.0
    for x in a:
        if b:
            worst = max(worst, min(abs(x - y) for y in b))
    print('Worst time difference: %.4f s' % worst)
rclpy.shutdown()
sys.exit(0)
