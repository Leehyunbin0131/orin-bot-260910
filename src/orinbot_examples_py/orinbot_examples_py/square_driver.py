#!/usr/bin/env python3
"""Odometry feedback based square driving example node (rclpy).

    ros2 run orinbot_examples_py square_driver --ros-args -p use_sim_time:=true
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(q) -> float:
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    """Normalize angle to [-pi, pi)."""
    return math.atan2(math.sin(a), math.cos(a))


class SquareDriver(Node):

    DRIVE = 'drive'
    TURN = 'turn'

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side_length', 1.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('position_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.02)

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tol = self.get_parameter('position_tol').value
        self.yaw_tol = self.get_parameter('yaw_tol').value

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, '/odom', self.on_odom, odom_qos)

        self.state = self.DRIVE
        self.leg = 0            # Number of completed legs
        self.pose = None        # (x, y, yaw)
        self.segment_start = None

        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz
        self.get_logger().info('square_driver started -- waiting for /odom')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def publish_cmd(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def on_timer(self):
        if self.pose is None:
            return

        if self.leg >= 4:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Square trajectory completed')
            self.timer.cancel()
            return

        x, y, yaw = self.pose

        if self.segment_start is None:
            self.segment_start = (x, y, yaw)
            if self.state == self.DRIVE:
                self.get_logger().info(f'[Leg {self.leg+1}/4] Driving forward ({self.side_length} m)...')
            else:
                self.get_logger().info(f'[Leg {self.leg+1}/4] Turning 90 degrees...')

        sx, sy, syaw = self.segment_start

        if self.state == self.DRIVE:
            traveled = math.hypot(x - sx, y - sy)
            remaining = self.side_length - traveled

            if remaining <= self.position_tol:
                self.publish_cmd(0.0, 0.0)
                self.state = self.TURN
                self.segment_start = None
                return

            speed = min(self.linear_speed, max(0.05, remaining * 1.0))
            self.publish_cmd(speed, 0.0)

        elif self.state == self.TURN:
            target_yaw = normalize_angle(syaw + math.pi / 2.0)
            diff = normalize_angle(target_yaw - yaw)

            if abs(diff) <= self.yaw_tol:
                self.publish_cmd(0.0, 0.0)
                self.leg += 1
                self.state = self.DRIVE
                self.segment_start = None
                return

            speed = min(self.angular_speed, max(0.1, abs(diff) * 1.5))
            if diff < 0:
                speed = -speed
            self.publish_cmd(0.0, speed)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
