#!/usr/bin/env python3
"""Script to verify /map based frontier detection algorithm execution."""
import time
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage


class P(Node):
    def __init__(self):
        super().__init__('verify_frontier')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.msg = None
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._cb, q)

    def _cb(self, m):
        self.msg = m


rclpy.init()
n = P()
t = time.time()
while n.msg is None and time.time() - t < 20:
    rclpy.spin_once(n, timeout_sec=0.2)

m = n.msg
i = m.info
res = i.resolution
g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
free = (g >= 0) & (g <= 25)
unk = g < 0
occ = g >= 65

nb = np.zeros_like(unk)
nb[1:, :] |= unk[:-1, :]
nb[:-1, :] |= unk[1:, :]
nb[:, 1:] |= unk[:, :-1]
nb[:, :-1] |= unk[:, 1:]
fr = free & nb
dist = ndimage.distance_transform_edt(~occ) * res
lbl, k = ndimage.label(fr, structure=np.ones((3, 3), bool))

print('Clusters passed min_cells=8 & min_clearance=0.18 out of %d:' % k)
kept = 0
for j in range(1, k + 1):
    ys, xs = np.nonzero(lbl == j)
    if len(xs) < 8:
        continue
    d = dist[ys, xs]
    if d.max() < 0.18:
        continue
    cx, cy = xs.mean(), ys.mean()
    ok = np.nonzero(d >= min(0.22, d.max()))[0]
    q = ok[np.argmin((xs[ok] - cx) ** 2 + (ys[ok] - cy) ** 2)]
    wx = i.origin.position.x + (xs[q] + 0.5) * res
    wy = i.origin.position.y + (ys[q] + 0.5) * res
    print('   (%6.2f, %6.2f)  cells: %3d  max clearance: %.2f m' % (wx, wy, len(xs), d.max()))
    kept += 1

print('=> Goal candidates: %d locations' % kept)
rclpy.shutdown()
