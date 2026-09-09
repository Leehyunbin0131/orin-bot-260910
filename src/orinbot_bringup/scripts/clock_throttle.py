#!/usr/bin/env python3
"""Downsampling node for Gazebo simulation `/clock` topic publish rate.

    /clock_raw (bridge) --> [ ClockThrottle ] --> /clock (default 100 Hz)

Reduces ROS `/clock` message frequency while preserving simulation physics steps to manage rclpy clock callback CPU load.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

# QoS profile matching ros_gz_bridge clock publication
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ClockThrottle(Node):

    def __init__(self):
        # Do NOT set use_sim_time on this node (it is the source of simulation time)
        super().__init__('clock_throttle')
        self.declare_parameter('rate', 100.0)
        rate = float(self.get_parameter('rate').value)

        self.latest = None
        self.pub = self.create_publisher(Clock, 'clock', QOS)
        self.create_subscription(Clock, 'clock_raw', self._in, QOS)

        self.passthrough = rate <= 0.0
        if rate > 0.0:
            # Wall-clock timer (use_sim_time=False)
            self.create_timer(1.0 / rate, self._tick)
            self.get_logger().info('Throttling /clock_raw -> /clock at %.0f Hz' % rate)
        else:
            self.get_logger().info('No clock throttling (passthrough mode)')

    def _in(self, msg):
        self.latest = msg
        if self.passthrough:
            self.pub.publish(msg)

    def _tick(self):
        if self.latest is not None:
            self.pub.publish(self.latest)


def main():
    rclpy.init()
    n = ClockThrottle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
