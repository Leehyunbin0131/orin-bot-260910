#!/usr/bin/env python3
"""Drive four odometry-based sides and four left 90-degree turns, then stop."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
import yaml
from ament_index_python.packages import get_package_share_directory

RAW_UNIT = 0.0239691227
STOP_PUBLISH_SECONDS = 6.0  # 60 rpm at 0.05 m/s² deceleration, plus feedback margin.


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def bounded_twist(linear, angular, radius, track, wheel_limit):
    """Keep each wheel below the requested RPM even during heading correction."""
    peak = max(abs(linear-angular*track/2), abs(linear+angular*track/2)) / radius
    scale = min(1.0, wheel_limit/peak) if peak else 1.0
    return linear*scale, angular*scale


class Rectangle(Node):
    def __init__(self, args):
        super().__init__('drive_rectangle')
        self.args = args
        settings = yaml.safe_load(args.config.read_text())
        self.radius, self.track = float(settings['wheel_radius']), float(settings['wheel_separation'])
        if not all(math.isfinite(v) and v > 0 for v in (self.radius, self.track)):
            raise ValueError('Invalid wheel dimensions')
        self.wheel_limit = math.floor(args.rpm / 0.229) * RAW_UNIT
        self.speed = self.wheel_limit * self.radius
        self.turn_rpm = min(getattr(args, 'turn_rpm', 10.0), args.rpm)
        self.turn_speed = 2 * math.floor(self.turn_rpm/0.229)*RAW_UNIT*self.radius/self.track
        self.turn_response_scale = max(1.0, self.turn_rpm/10.0)
        self.pose = None
        self.velocity = (0.0, 0.0)
        self.last_odom = 0.0
        self.last_stamp = None
        self.odom_error = None
        self.description = None
        self.phase = 'waiting'
        self.last_log = 0.0
        self.last_command = (0.0, 0.0)
        self.directory = Path.home() / 'ros2_ws/log' / getattr(args, 'log_subdir', 'rectangle_runs') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        self.events = (self.directory/'events.jsonl').open('w')
        self.summary = {'width_m': args.width, 'height_m': args.height, 'wheel_rpm': args.rpm,
                        'turn_wheel_rpm': self.turn_rpm, 'corners': [], 'status': 'fail'}
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, getattr(args, 'odom_topic', '/odom'), self.odom_callback, 10)
        self.description_sub = self.create_subscription(
            String, '/robot_description', lambda m: setattr(self, 'description', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def record(self, kind, **values):
        self.events.write(json.dumps({'time': time.time(), 'kind': kind, 'phase': self.phase, **values},
                                     allow_nan=False) + '\n')
        self.events.flush()

    def odom_callback(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = (p.x, p.y, yaw, message.twist.twist.linear.x, message.twist.twist.angular.z)
        stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
        if not all(math.isfinite(v) for v in values):
            self.odom_error = 'Non-finite odometry'
            return
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_footprint':
            self.odom_error = 'Unexpected odometry frames'
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.odom_error = 'Odometry timestamp stopped or moved backwards'
        if self.pose:
            yaw = self.pose[2] + wrap(yaw-self.pose[2])
        self.pose = (p.x, p.y, yaw)
        self.velocity = values[3:]
        self.last_stamp, self.last_odom = stamp, time.monotonic()
        if self.last_odom-self.last_log >= 0.1:
            self.record('odom', x=p.x, y=p.y, yaw=yaw, linear=self.velocity[0],
                        angular=self.velocity[1], command=self.last_command)
            self.last_log = self.last_odom

    def send(self, linear=0.0, angular=0.0):
        linear, angular = bounded_twist(linear, angular, self.radius, self.track, self.wheel_limit)
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x, message.twist.angular.z = linear, angular
        self.publisher.publish(message)
        self.last_command = (linear, angular)

    def tick(self, linear=0.0, angular=0.0):
        if self.odom_error or time.monotonic()-self.last_odom > 0.5:
            raise RuntimeError(self.odom_error or 'Odometry missing for 0.5 seconds')
        if self.count_publishers('/cmd_vel') != 1:
            raise RuntimeError('Another /cmd_vel publisher is active')
        self.send(linear, angular)
        deadline = time.monotonic()+0.05
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=max(0, deadline-time.monotonic()))

    def wait_ready(self):
        deadline = time.monotonic()+25
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose and self.description and self.publisher.get_subscription_count():
                break
        if not self.pose or not self.description or not self.publisher.get_subscription_count():
            raise RuntimeError('Base is not ready: /odom, robot_description or /cmd_vel subscriber missing')
        is_mock = 'mock_components/GenericSystem' in self.description
        if is_mock != self.args.mock:
            raise RuntimeError('Requested mock/real mode does not match the hardware plugin')
        if not is_mock and 'dynamixel_hardware_interface/DynamixelHardware' not in self.description:
            raise RuntimeError('Unexpected hardware plugin')
        self.summary['mock_hardware'] = is_mock
        client = self.create_client(GetParameters, '/diff_drive_controller/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Controller parameter service missing')
        names = ['linear.x.max_velocity', 'angular.z.max_velocity', 'wheel_radius', 'wheel_separation',
                 'cmd_vel_timeout', 'open_loop']
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done() or future.exception():
            raise RuntimeError('Controller parameter query failed')
        p = future.result().values
        if (p[0].double_value+1e-9 < self.speed or p[1].double_value+1e-9 < self.turn_speed
                or not math.isclose(p[2].double_value, self.radius)
                or not math.isclose(p[3].double_value, self.track)
                or p[4].double_value > 0.3 or p[5].bool_value):
            raise RuntimeError('Use rectangle.launch.py with matching rpm and wheel configuration')
        self.settle()

    def settle(self):
        deadline = time.monotonic()+6
        stable = None
        while time.monotonic() < deadline:
            self.tick()
            if abs(self.velocity[0]) < 0.003 and abs(self.velocity[1]) < 0.01:
                stable = stable or time.monotonic()
                if time.monotonic()-stable >= 0.3:
                    return
            else:
                stable = None
        raise RuntimeError('Robot did not stop')

    def segment(self, length, heading, turning=False):
        origin = self.pose
        initial_error = abs(heading-origin[2]) if turning else length
        maximum = self.turn_speed if turning else self.speed
        deadline = time.monotonic()+initial_error/maximum*2+20
        best, progress_time = initial_error, time.monotonic()
        while time.monotonic() < deadline:
            x, y, yaw = self.pose
            if turning:
                error = heading-yaw
                if abs(error) < 0.015:
                    break
                angular = math.copysign(min(self.turn_speed,
                    0.8*self.turn_response_scale*abs(error),
                    math.sqrt(2*0.08*self.turn_response_scale*abs(error))), error)
                self.tick(0.0, angular)
                remaining = abs(error)
            else:
                dx, dy = x-origin[0], y-origin[1]
                forward = math.cos(heading)*dx + math.sin(heading)*dy
                lateral = -math.sin(heading)*dx + math.cos(heading)*dy
                remaining = length-forward
                if remaining < 0.015:
                    break
                if abs(lateral) > 0.20:
                    raise RuntimeError('Lateral deviation exceeds 20 cm')
                linear = min(self.speed, 0.8*remaining, math.sqrt(2*0.04*remaining))
                angular = max(-self.turn_speed, min(self.turn_speed, 1.5*wrap(heading-yaw)-0.8*lateral))
                self.tick(linear, angular)
            if remaining < best-0.002:
                best, progress_time = remaining, time.monotonic()
            elif time.monotonic()-progress_time > 5:
                raise RuntimeError('No progress for 5 seconds')
        else:
            raise RuntimeError('Segment time limit exceeded')
        self.settle()

    def run(self):
        self.wait_ready()
        start = self.pose
        self.summary['start_pose'] = start
        print(f'시작: {self.args.width}m × {self.args.height}m, '
              f'직진 {self.args.rpm}rpm / 회전 바퀴 {self.turn_rpm}rpm 상한', flush=True)
        for i, length in enumerate((self.args.width, self.args.height)*2):
            heading = start[2]+i*math.pi/2
            self.phase = f'side_{i+1}'
            print(f'{i+1}/4: {length}m 전진', flush=True)
            self.segment(length, heading)
            corner = {'side': i+1, 'pose': self.pose}
            self.summary['corners'].append(corner)
            self.record('corner', **corner)
            self.phase = f'turn_{i+1}'
            print('왼쪽으로 90도 회전', flush=True)
            self.segment(0, heading+math.pi/2, turning=True)
        self.summary.update(status='pass', final_pose=self.pose,
                            closure_error_m=math.hypot(self.pose[0]-start[0], self.pose[1]-start[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=float, default=2.0)
    parser.add_argument('--height', type=float, default=2.0)
    parser.add_argument('--rpm', type=float, default=50.0)
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--config', type=Path, default=Path(get_package_share_directory('orinbot_hardware'))/'config/hardware.yaml')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) and 0.1 <= v <= 5 for v in (args.width, args.height)):
        parser.error('Side lengths must be 0.1..5 metres')
    if not math.isfinite(args.rpm) or not 1 <= args.rpm <= 50:
        parser.error('Wheel rpm must be 1..50')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Rectangle(args)
    print(f'로그: {node.directory}', flush=True)
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        node.summary.update(status='fail', reason=str(exc) or 'Interrupted')
        print(f'중단: {node.summary["reason"]}', flush=True)
    finally:
        # Keep publishing zero through deceleration and encoder feedback delay.
        node.phase = 'stopping'
        deadline = time.monotonic()+STOP_PUBLISH_SECONDS
        while time.monotonic() < deadline and rclpy.ok():
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.summary['stopped'] = (time.monotonic()-node.last_odom < 0.5
                                   and abs(node.velocity[0]) < 0.003 and abs(node.velocity[1]) < 0.01)
        if not node.summary['stopped']:
            node.summary.update(status='fail', reason='Final stop feedback not confirmed')
        (node.directory/'summary.json').write_text(json.dumps(node.summary, indent=2)+'\n')
        node.events.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f'결과: {node.summary["status"].upper()} / 로그: {node.directory}', flush=True)
    return 0 if node.summary['status'] == 'pass' else 1


if __name__ == '__main__':
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    sys.exit(main())
