#!/usr/bin/env python3
"""Nonblocking, bounded encoder moves. Service responses acknowledge acceptance only."""
import json
import math
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orinbot_hardware.srv import Move
from ament_index_python.packages import get_package_share_directory
import yaml
from drive_rectangle import bounded_twist


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def validate_move(distance_cm, angle_deg):
    if not all(math.isfinite(v) for v in (distance_cm, angle_deg)):
        raise ValueError('Values must be finite')
    if (distance_cm != 0) == (angle_deg != 0):
        raise ValueError('Set exactly one of distance_cm or angle_deg')
    if distance_cm and not 1 <= abs(distance_cm) <= 200:
        raise ValueError('Distance magnitude must be 1..200 cm')
    if angle_deg and not 1 <= abs(angle_deg) <= 360:
        raise ValueError('Angle magnitude must be 1..360 degrees')
    return ('distance', distance_cm/100) if distance_cm else ('angle', math.radians(angle_deg))


class MotionService(Node):
    def __init__(self):
        super().__init__('motion_service')
        self.mock = self.declare_parameter('mock_hardware', False).value
        settings = yaml.safe_load((Path(get_package_share_directory('orinbot_hardware'))/'config/hardware.yaml').read_text())
        self.radius, self.track = float(settings['wheel_radius']), float(settings['wheel_separation'])
        self.wheel_limit = math.floor(10/0.229)*0.0239691227
        self.pose = None
        self.velocity = (0.0, 0.0)
        self.last_odom = 0.0
        self.last_stamp = None
        self.odom_error = None
        self.description = ''
        self.state = 'idle'
        self.command_id = 0
        self.origin = None
        self.target = 0.0
        self.kind = None
        self.reason = 'Waiting for base feedback'
        self.result_state = 'idle'
        self.stable = None
        self.last_command = (0.0, 0.0)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_description', self.description_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Move, '/robot/move', self.move)
        self.create_service(Trigger, '/robot/stop', self.stop)
        self.create_service(Trigger, '/robot/status', self.status)
        self.create_timer(0.05, self.tick)
        directory = Path.home()/'ros2_ws/log/motion_service'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True)
        self.log = (directory/'events.jsonl').open('w')
        self.get_logger().info(f'Services: /robot/move, /robot/stop, /robot/status; log: {directory}')

    def record(self, event):
        self.log.write(json.dumps({'time': time.time(), 'event': event, **self.snapshot()}, allow_nan=False)+'\n')
        self.log.flush()

    def description_callback(self, msg):
        self.description = msg.data

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[2:6])-1) > 0.01:
            self.odom_error = 'Invalid odometry values'; return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            self.odom_error = 'Unexpected odometry frames'; return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.odom_error = 'Odometry timestamp did not advance'; return
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if age > 0.5 or age < -0.1:
            self.odom_error = 'Odometry timestamp is stale or in the future'; return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if self.pose:
            delta = wrap(yaw-self.pose[2])
            if math.hypot(p.x-self.pose[0], p.y-self.pose[1]) > 0.1 or abs(delta) > 0.25:
                self.odom_error = 'Odometry jumped'; return
            yaw = self.pose[2]+delta
        self.pose = (p.x, p.y, yaw)
        self.velocity = values[-2:]
        self.last_stamp, self.last_odom = stamp, time.monotonic()

    def problem(self):
        if self.odom_error:
            return self.odom_error+'; restart service after fixing feedback'
        if self.pose is None or time.monotonic()-self.last_odom > 0.5:
            return 'Odometry missing for 0.5 seconds'
        plugin = 'mock_components/GenericSystem' if self.mock else 'dynamixel_hardware_interface/DynamixelHardware'
        if plugin not in self.description or (('mock_components/GenericSystem' in self.description) != self.mock):
            return 'Hardware description missing or mock/real mode mismatch'
        if self.count_publishers('/odom') != 1:
            return 'Expected exactly one odometry publisher'
        if self.count_publishers('/cmd_vel') != 1:
            return 'Another velocity publisher is active; stop other driving programs'
        if not self.publisher.get_subscription_count():
            return 'Base command subscriber missing'
        return ''

    def send(self, linear=0.0, angular=0.0):
        linear, angular = bounded_twist(linear, angular, self.radius, self.track, self.wheel_limit)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        self.publisher.publish(msg)
        self.last_command = (linear, angular)

    def progress(self):
        if self.origin is None or self.pose is None:
            return 0.0, 0.0
        dx, dy = self.pose[0]-self.origin[0], self.pose[1]-self.origin[1]
        return (math.cos(self.origin[2])*dx+math.sin(self.origin[2])*dy,
                self.pose[2]-self.origin[2])

    def snapshot(self):
        distance, angle = self.progress()
        return {'command_id': self.command_id, 'state': self.state, 'message': self.reason,
                'distance_cm': round(distance*100, 3), 'angle_deg': round(math.degrees(angle), 3)}

    def move(self, request, response):
        response.command_id = self.command_id
        try:
            kind, target = validate_move(request.distance_cm, request.angle_deg)
            if self.state in ('moving', 'stopping'):
                raise ValueError('Busy: wait for completion or call /robot/stop')
            issue = self.problem()
            if issue:
                raise ValueError(issue)
            if abs(self.velocity[0]) > 0.001 or abs(self.velocity[1]) > 0.005:
                raise ValueError('Robot is not stationary')
            self.command_id += 1
            self.kind, self.target, self.origin = kind, target, self.pose
            now = time.monotonic()
            self.deadline = now+abs(target)/(0.03 if kind == 'distance' else 0.12)*3+15
            self.best, self.progress_time = abs(target), now
            self.state, self.reason = 'moving', 'Command accepted; query /robot/status for completion'
            self.record('accepted')
            response.accepted, response.command_id, response.message = True, self.command_id, self.reason
        except ValueError as exc:
            response.accepted, response.message = False, str(exc)
        return response

    def begin_stop(self, state, reason):
        self.state, self.result_state, self.reason = 'stopping', state, reason
        self.stop_deadline, self.stable = time.monotonic()+6, None
        self.send()
        self.record('stopping')

    def stop(self, request, response):
        self.begin_stop('cancelled', 'Stop requested; waiting for stationary feedback')
        response.success, response.message = True, 'Stop requested; query /robot/status for confirmed stop'
        return response

    def status(self, request, response):
        data = self.snapshot()
        issue = self.problem()
        if not issue and (abs(self.velocity[0]) > 0.001 or abs(self.velocity[1]) > 0.005):
            issue = 'Robot is not stationary'
        data.update(ready=not issue and self.state not in ('moving', 'stopping'), readiness_issue=issue)
        response.success, response.message = True, json.dumps(data, ensure_ascii=False)
        return response

    def tick(self):
        now = time.monotonic()
        if self.state == 'stopping':
            self.send()
            fresh = self.pose is not None and now-self.last_odom <= 0.5 and not self.odom_error
            if fresh and abs(self.velocity[0]) < 0.001 and abs(self.velocity[1]) < 0.005:
                self.stable = self.stable or now
                if now-self.stable >= 0.4:
                    self.state = self.result_state
                    if self.state == 'succeeded':
                        d, a = self.progress()
                        error = abs(self.target-(d if self.kind == 'distance' else a))
                        if error > (0.005 if self.kind == 'distance' else math.radians(1)):
                            self.state, self.reason = 'failed', 'Stopped outside target tolerance'
                        else:
                            self.reason = 'Target reached and stop confirmed (encoder odometry)'
                    self.record('finished')
                    return
            else:
                self.stable = None
            if now > self.stop_deadline:
                self.state, self.reason = 'failed', 'Stop sent; stationary feedback could not be confirmed'
                self.record('finished')
            return
        if self.state != 'moving':
            return
        issue = self.problem()
        if issue:
            self.begin_stop('failed', issue); return
        distance, angle = self.progress()
        error = self.target-(distance if self.kind == 'distance' else angle)
        remaining = abs(error)
        if remaining <= (0.002 if self.kind == 'distance' else math.radians(0.4)):
            self.begin_stop('succeeded', 'Target reached; waiting for stop'); return
        if now > self.deadline:
            self.begin_stop('failed', 'Motion time limit exceeded'); return
        if remaining < self.best-(0.0005 if self.kind == 'distance' else 0.002):
            self.best, self.progress_time = remaining, now
        elif now-self.progress_time > 5:
            self.begin_stop('failed', 'No encoder progress for 5 seconds'); return
        if self.kind == 'angle':
            if abs(distance) > 0.05:
                self.begin_stop('failed', 'Unexpected translation during rotation'); return
            self.send(0.0, math.copysign(min(0.12, 0.8*remaining), error))
        else:
            dx, dy = self.pose[0]-self.origin[0], self.pose[1]-self.origin[1]
            lateral = -math.sin(self.origin[2])*dx+math.cos(self.origin[2])*dy
            if abs(lateral) > 0.05 or abs(angle) > math.radians(20):
                self.begin_stop('failed', 'Excessive lateral or heading deviation'); return
            self.send(math.copysign(min(0.03, 0.8*remaining), error), max(-0.04, min(0.04, -1.2*angle)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MotionService()
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep zero commands during deceleration; base also has its own command watchdog.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.begin_stop('cancelled', 'Service shutting down')
        deadline = time.monotonic()+2
        while rclpy.ok() and time.monotonic() < deadline:
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
