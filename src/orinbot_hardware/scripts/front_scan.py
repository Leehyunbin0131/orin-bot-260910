#!/usr/bin/env python3
"""Mask the rear sector without changing LaserScan ray ordering or timing."""
import math


def mask_scan(scan, yaw_degrees, fov_degrees):
    if not math.isfinite(yaw_degrees) or not math.isfinite(fov_degrees) or not 0 < fov_degrees <= 360:
        raise ValueError('Finite mount yaw and field of view in (0, 360] required')
    if scan.header.frame_id != 'laser':
        raise ValueError('Expected laser frame; mounting yaw would be invalid for another frame')
    if not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        raise ValueError('Invalid scan angles')
    half = math.radians(fov_degrees) / 2
    yaw = math.radians(yaw_degrees)
    for index in range(len(scan.ranges)):
        angle = scan.angle_min + index * scan.angle_increment + yaw
        body_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(body_angle) > half + 1e-7:
            # NaN means unobserved. +inf would incorrectly clear rear obstacles
            # for consumers with inf_is_valid enabled.
            scan.ranges[index] = math.nan
            if index < len(scan.intensities):
                scan.intensities[index] = 0.0
    return scan


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import LaserScan

    class FrontScan(Node):
        def __init__(self):
            super().__init__('front_scan')
            self.yaw = float(self.declare_parameter('mount_yaw_degrees', 180.0).value)
            self.fov = float(self.declare_parameter('fov_degrees', 270.0).value)
            if not math.isfinite(self.yaw) or not math.isfinite(self.fov) or not 0 < self.fov <= 360:
                raise ValueError('Invalid LiDAR mounting yaw or field of view')
            # Reliable output serves both reliable and best-effort subscribers
            # (ICP, SLAM, Nav2 and diagnostic clients).
            self.publisher = self.create_publisher(LaserScan, '/scan', 5)
            self.subscription = self.create_subscription(LaserScan, '/scan_raw', self.receive, qos_profile_sensor_data)

        def receive(self, scan):
            try:
                self.publisher.publish(mask_scan(scan, self.yaw, self.fov))
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5)

    rclpy.init()
    node = FrontScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
