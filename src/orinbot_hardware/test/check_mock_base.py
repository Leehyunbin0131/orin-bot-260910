#!/usr/bin/env python3
"""Integration check; run only against base.launch.py mock_hardware:=true on domain 174."""
import math
import os
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

if os.environ.get('ROS_DOMAIN_ID') != '174':
    raise RuntimeError('This mock-only check requires ROS_DOMAIN_ID=174')
rclpy.init()
node = rclpy.create_node('orinbot_mock_check')
state = {}
subscriptions = [
    node.create_subscription(String, '/robot_description', lambda m: state.update(description=m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 10),
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(joints=m), 10),
]
publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)


def spin_for(seconds, velocity=None, turn=0.0, stale=False):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if velocity is not None:
            message = TwistStamped()
            stamp = node.get_clock().now().to_msg()
            if stale:
                stamp.sec -= 2
            message.header.stamp = stamp
            message.header.frame_id = 'base_footprint'
            message.twist.linear.x = velocity
            message.twist.angular.z = turn
            publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.025)
        time.sleep(0.025)


try:
    deadline = time.monotonic() + 15
    while not all(key in state for key in ('description', 'odom', 'joints')) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert all(key in state for key in ('description', 'odom', 'joints')), 'Base did not become ready'
    assert 'mock_components/GenericSystem' in state['description'], 'Refusing commands to real hardware'
    assert 'dynamixel_hardware_interface/DynamixelHardware' not in state['description']
    spin_for(2.0, velocity=0.8)
    speed = state['odom'].twist.twist.linear.x
    assert 0.0059 <= speed <= 0.0061, f'Linear speed limit failed: {speed}'
    joints = dict(zip(state['joints'].name, state['joints'].velocity))
    for name in ('left_wheel_joint', 'right_wheel_joint'):
        assert math.isclose(joints[name], speed / 0.0325, rel_tol=0.02), (name, joints)
    print(f'PASS: linear speed capped at {speed:.3f} m/s; wheel radius conversion correct')
    spin_for(1.2)
    assert abs(state['odom'].twist.twist.linear.x) < 0.001, 'Command timeout did not stop base'
    print('PASS: command loss stops base')
    spin_for(1.8, velocity=0.0, turn=1.0)
    yaw_rate = state['odom'].twist.twist.angular.z
    assert 0.019 <= yaw_rate <= 0.0201, f'Angular limit failed: {yaw_rate}'
    joints = dict(zip(state['joints'].name, state['joints'].velocity))
    expected = yaw_rate * 0.50 / (2 * 0.0325)
    assert math.isclose(joints['left_wheel_joint'], -expected, rel_tol=0.02), joints
    assert math.isclose(joints['right_wheel_joint'], expected, rel_tol=0.02), joints
    print(f'PASS: angular speed capped at {yaw_rate:.3f} rad/s; 0.50 m track conversion correct')
    spin_for(1.2)
    spin_for(1.0, velocity=0.8, stale=True)
    assert abs(state['odom'].twist.twist.linear.x) < 0.001, 'Stale command was accepted'
    assert abs(state['odom'].twist.twist.angular.z) < 0.001, 'Turn did not stop'
    print('PASS: stale timestamp rejected')
finally:
    node.destroy_node()
    rclpy.shutdown()
