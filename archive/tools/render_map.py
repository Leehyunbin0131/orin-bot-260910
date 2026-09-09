#!/usr/bin/env python3
"""Script to render /map occupancy grid data to PNG image with frontier visualization."""
import time
import numpy as np
from PIL import Image
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage


class P(Node):
    def __init__(self):
        super().__init__('render_map')
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
while n.msg is None and time.time() - t < 30:
    rclpy.spin_once(n, timeout_sec=0.2)

m = n.msg
i = m.info
g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
free = (g >= 0) & (g <= 25)
unk = g < 0
occ = g >= 65

img = np.zeros((i.height, i.width, 3), np.uint8)
img[unk] = (110, 110, 130)
img[free] = (245, 245, 245)
img[occ] = (20, 20, 20)

nb = np.zeros_like(unk)
nb[1:, :] |= unk[:-1, :]
nb[:-1, :] |= unk[1:, :]
nb[:, 1:] |= unk[:, :-1]
nb[:, :-1] |= unk[:, 1:]
fr = free & nb
img[fr] = (255, 40, 40)
gr = ndimage.binary_dilation(occ, structure=np.ones((3, 3), bool), iterations=5)
img[fr & ~gr] = (0, 200, 0)
Image.fromarray(np.flipud(img)).resize((i.width * 3, i.height * 3), Image.NEAREST).save('explore_map_turn.png')

print('Origin (%.2f, %.2f) %dx%d -- Red=Frontier (%d), Green=Clearance-passed (%d)'
      % (i.origin.position.x, i.origin.position.y, i.width, i.height, fr.sum(), (fr & ~gr).sum()))
rclpy.shutdown()
