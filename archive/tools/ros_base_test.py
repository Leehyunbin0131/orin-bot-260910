#!/usr/bin/env python3
"""Bounded ROS base check, owning its launch process and timestamped logs."""
import argparse
from datetime import datetime
import importlib.util
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import DynamicJointState
from dynamixel_interfaces.msg import DynamixelState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/orinbot_hardware'
support_spec = importlib.util.spec_from_file_location(
    'orinbot_interface_test_support', PACKAGE / 'scripts/test_interfaces.py')
support = importlib.util.module_from_spec(support_spec)
support_spec.loader.exec_module(support)
MotorBus, Recorder = support.MotorBus, support.Recorder
port_problem, stop_motors = support.port_problem, support.stop_motors

spec = importlib.util.spec_from_file_location('base_launch', PACKAGE / 'launch/base.launch.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
WHEELS = ('left_wheel_joint', 'right_wheel_joint')
RAW_UNIT = 0.0239691227


class BaseCheck:
    def __init__(self, args, recorder, settings):
        self.args, self.recorder, self.settings = args, recorder, settings
        self.raw_limit = math.floor(args.rpm_sweep / 0.229) if args.rpm_sweep else 15
        self.context = Context()
        rclpy.init(context=self.context, domain_id=args.domain)
        self.node = rclpy.create_node('orinbot_ros_base_test', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.state, self.received, self.counts = {}, {}, {}
        self.first_received = {}
        self.phase, self.monitor = 'startup', False
        self.max_wheel_speed = 0.0
        self.subscriptions = []
        for kind, msg_type, topic, qos in (
                ('description', String, '/robot_description',
                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                ('joints', JointState, '/joint_states', 10),
                ('dynamic', DynamicJointState, '/dynamic_joint_states', 10),
                ('odom', Odometry, '/odom', 10),
                ('dxl', DynamixelState, '/dynamixel_hardware_interface/dxl_state', 10),
                ('tf', TFMessage, '/tf', 30),
                ('tf_static', TFMessage, '/tf_static',
                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))):
            self.subscriptions.append(self.node.create_subscription(
                msg_type, topic, lambda m, k=kind: self.receive(k, m), qos))
        self.publisher = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.process = None
        self.last_command_time = None

    def receive(self, kind, message):
        now = time.monotonic()
        self.state[kind], self.received[kind] = message, now
        self.first_received.setdefault(kind, now)
        self.counts[kind] = self.counts.get(kind, 0) + 1
        data = {}
        if kind == 'joints':
            data = {'names': list(message.name), 'position': list(message.position),
                    'velocity': list(message.velocity)}
        elif kind == 'odom':
            p, q = message.pose.pose.position, message.pose.pose.orientation
            data = {'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                    'linear': message.twist.twist.linear.x, 'angular': message.twist.twist.angular.z,
                    'frame': message.header.frame_id, 'child': message.child_frame_id}
        elif kind == 'dxl':
            data = {'ids': list(message.id), 'driver_reported_torque': list(message.torque_state),
                    'communication': message.comm_state, 'hardware_errors': list(message.dxl_hw_state)}
        elif kind == 'dynamic':
            data = {'interfaces': {name: dict(zip(values.interface_names, values.values))
                                   for name, values in zip(message.joint_names, message.interface_values)}}
        elif kind in ('tf', 'tf_static'):
            for transform in message.transforms:
                self.state[('transform', transform.child_frame_id)] = transform
            data = {'frames': [(t.header.frame_id, t.child_frame_id) for t in message.transforms]}
        if data:
            if hasattr(message, 'header'):
                data['stamp_ns'] = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            self.recorder.event(kind, phase=self.phase, **data)

    def joint_values(self, field):
        message = self.state['joints']
        values = dict(zip(message.name, getattr(message, field)))
        return [values[name] for name in WHEELS]

    def motor_states(self):
        message = self.state['dynamic']
        values = {name: dict(zip(v.interface_names, v.values))
                  for name, v in zip(message.joint_names, message.interface_values)}
        return [values[name] for name in ('left_motor', 'right_motor')]

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.process and self.process.poll() is not None:
                raise RuntimeError('Base launch exited; inspect base.log')
            self.executor.spin_once(timeout_sec=min(0.02, max(0, deadline-time.monotonic())))
            if self.monitor:
                required = ['joints', 'odom'] + ([] if self.args.mode == 'mock' else ['dxl', 'dynamic'])
                if any(time.monotonic() - self.received.get(k, 0) > 0.8 for k in required):
                    raise RuntimeError('ROS feedback missing for more than 0.8 seconds')
                velocities = self.joint_values('velocity')
                positions = self.joint_values('position')
                if not all(math.isfinite(v) for v in velocities + positions):
                    raise RuntimeError('Non-finite wheel feedback')
                self.max_wheel_speed = max(self.max_wheel_speed, *map(abs, velocities))
                measured_limit = self.raw_limit + max(2, self.raw_limit * 0.15)
                if any(abs(v) > measured_limit * RAW_UNIT for v in velocities):
                    raise RuntimeError('Measured wheel speed exceeded low-speed tolerance')
                if self.args.mode != 'mock':
                    dxl = self.state['dxl']
                    if dxl.comm_state or any(dxl.dxl_hw_state):
                        raise RuntimeError('Dynamixel communication or hardware error')
                    for values in self.motor_states():
                        if not math.isfinite(values['Goal Velocity']) or abs(values['Goal Velocity']) > self.raw_limit * RAW_UNIT + 1e-6:
                            raise RuntimeError('Motor goal register exceeded requested speed limit')

    def publish(self, linear=0.0, angular=0.0, stale=False):
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        if stale:
            message.header.stamp.sec -= 2
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x, message.twist.angular.z = linear, angular
        self.publisher.publish(message)
        self.last_command_time = time.monotonic()
        self.recorder.event('command', phase=self.phase, linear=linear, angular=angular, stale=stale)

    def command_for(self, duration, linear=0.0, angular=0.0, stale=False):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish(linear, angular, stale)
            self.spin(0.05)

    def check_stopped(self):
        velocity = self.joint_values('velocity')
        odom = self.state['odom'].twist.twist
        return (all(abs(v) <= 2 * RAW_UNIT for v in velocity)
                and abs(odom.linear.x) < 0.001 and abs(odom.angular.z) < 0.004)

    def wait_for_stop(self, timeout=1.8):
        origin = self.last_command_time
        deadline = origin + timeout
        stopped_since = None
        while time.monotonic() < deadline:
            self.spin(0.025)
            if self.check_stopped():
                if stopped_since is None:
                    stopped_since = time.monotonic()
                if time.monotonic() - stopped_since >= 0.25:
                    return stopped_since - origin
            else:
                stopped_since = None
        raise RuntimeError(f'Command loss did not stop the wheels within {timeout} seconds')

    def ready(self):
        required = ['description', 'joints', 'odom', 'tf_static']
        if self.args.mode != 'mock':
            required.extend(['dxl', 'dynamic'])
        deadline = time.monotonic() + 25
        while not all(k in self.state for k in required) and time.monotonic() < deadline:
            self.spin(0.1)
        if not all(k in self.state for k in required):
            raise RuntimeError(f'Base startup incomplete: received {list(self.counts)}')
        description = self.state['description'].data
        expected = ('mock_components/GenericSystem' if self.args.mode == 'mock'
                    else 'dynamixel_hardware_interface/DynamixelHardware')
        if expected not in description:
            raise RuntimeError('Unexpected hardware plugin; refusing commands')
        (self.recorder.directory / 'robot_description.urdf').write_text(description)
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Controller manager service missing')
        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            self.spin(0.05)
        if not future.done() or future.exception():
            raise RuntimeError('Controller status query failed')
        controllers = {c.name: c.state for c in future.result().controller}
        if any(controllers.get(name) != 'active' for name in ('diff_drive_controller', 'joint_state_broadcaster')):
            raise RuntimeError(f'Controllers not active: {controllers}')
        self.recorder.result('ros_startup', 'pass', controllers=controllers, hardware_plugin=expected)
        parameter_client = self.node.create_client(GetParameters, '/diff_drive_controller/get_parameters')
        if not parameter_client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Controller parameter service missing')
        names = ['linear.x.max_velocity', 'linear.x.min_velocity', 'angular.z.max_velocity',
                 'angular.z.min_velocity', 'wheel_radius', 'wheel_separation', 'cmd_vel_timeout', 'open_loop']
        future = parameter_client.call_async(GetParameters.Request(names=names))
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            self.spin(0.05)
        if not future.done() or future.exception():
            raise RuntimeError('Controller parameter query failed')
        values = {name: (v.bool_value if v.type == 1 else v.double_value)
                  for name, v in zip(names, future.result().values)}
        base.validate_controller_limits(self.settings, {**values, 'linear.x.has_velocity_limits': True,
                                                       'angular.z.has_velocity_limits': True}, self.raw_limit)
        if values['open_loop'] or values['cmd_vel_timeout'] > 0.3:
            raise RuntimeError('Controller must use encoder feedback and a short command timeout')
        self.recorder.report['runtime_controller_parameters'] = values
        self.monitor = True
        self.spin(0.5)
        if not self.check_stopped():
            raise RuntimeError('Wheels moving before test commands')
        if self.args.mode != 'mock':
            dxl = self.state['dxl']
            if set(dxl.id) != {self.settings['left_id'], self.settings['right_id']}:
                raise RuntimeError('Unexpected motor feedback IDs')
            torque_registers = [values['Torque Enable'] for values in self.motor_states()]
            if any(t != int(self.args.mode == 'motion') for t in torque_registers):
                raise RuntimeError('Unexpected initial torque state')
            self.recorder.result('torque_register_feedback', 'pass', torque_registers=torque_registers,
                                 reason='Torque verified from register data in /dynamic_joint_states')
        odom = self.state['odom']
        if odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_footprint':
            raise RuntimeError('Unexpected odometry frame names')
        transform = self.state.get(('transform', 'base_footprint'))
        if transform is None or transform.header.frame_id != 'odom':
            raise RuntimeError('Missing odom -> base_footprint TF')
        for sensor, frame in (('lidar', 'laser'), ('camera', 'camera_link')):
            t = self.state.get(('transform', frame))
            if t is None or t.header.frame_id != 'base_footprint':
                raise RuntimeError(f'Missing sensor mount TF: {frame}')
            for axis in 'xyz':
                if not math.isclose(getattr(t.transform.translation, axis), self.settings[f'{sensor}_{axis}']):
                    raise RuntimeError(f'Incorrect sensor mount: {frame}/{axis}')
        self.recorder.result('feedback_and_tf', 'pass', reason='Wheel states, odom and measured sensor mounts received')

    def run_rpm_sweep(self):
        """One forward run with bounded speed plateaus, then controlled stop."""
        target = self.args.rpm_sweep
        steps = sorted({min(target, value) for value in (15, 30, 45, 60)})
        baseline = self.joint_values('position')
        for rpm in steps:
            raw = math.floor(rpm / 0.229)
            wheel_speed = raw * RAW_UNIT
            linear = wheel_speed * self.settings['wheel_radius']
            self.phase = f'speed_{rpm:g}_rpm'
            print(f'양륜 전진: {rpm:g} rpm 목표 (원시값 {raw})', flush=True)
            start = time.monotonic()
            samples = []
            while time.monotonic() - start < 3.0:
                self.publish(linear)
                self.spin(0.05)
                if time.monotonic() - start >= 2.0:
                    samples.append([v * 60 / (2 * math.pi) for v in self.joint_values('velocity')])
            if len(samples) < 5:
                raise RuntimeError('Insufficient steady speed feedback')
            actual = [statistics.median(s[i] for s in samples) for i in (0, 1)]
            plateau_target = raw * 0.229
            if any(abs(v-plateau_target) > max(2.0, plateau_target * 0.15) for v in actual):
                raise RuntimeError(f'{rpm:g} rpm: measured speeds {actual}; target tracking failed')
            self.recorder.result(self.phase, 'pass', requested_rpm=rpm, goal_raw=raw,
                                 quantized_rpm=plateau_target, median_rpm=actual,
                                 peak_rpm=[max(s[i] for s in samples) for i in (0, 1)],
                                 reason=f'측정 왼쪽 {actual[0]:.2f} / 오른쪽 {actual[1]:.2f} rpm')
        self.phase = 'speed_stop'
        self.publish(0.0)
        stopping = self.wait_for_stop(timeout=6.0)
        delta = [end-start for start, end in zip(baseline, self.joint_values('position'))]
        self.recorder.result('speed_stop', 'pass', stop_seconds=stopping,
                             wheel_revolutions=[v/(2*math.pi) for v in delta],
                             reason='속도 0 명령 후 양쪽 바퀴 정지 확인')

    def run_motion(self):
        radius, track = self.settings['wheel_radius'], self.settings['wheel_separation']
        for name, linear, angular in (('forward', 0.006, 0.0), ('reverse', -0.006, 0.0),
                                     ('turn_left', 0.0, 0.020), ('turn_right', 0.0, -0.020),
                                     ('combined', 0.006, 0.020)):
            self.phase = name
            before = self.joint_values('position')
            before_odom = self.state['odom']
            # Only the verified mock plugin receives deliberately excessive inputs.
            scale = 50 if self.args.mode == 'mock' else 1
            self.command_for(2.0, linear * scale, angular * scale)
            measured = self.joint_values('velocity')
            odom = self.state['odom'].twist.twist
            expected = [(linear - angular * track / 2) / radius,
                        (linear + angular * track / 2) / radius]
            for actual, target in zip(measured, expected):
                if abs(actual - target) > max(2 * RAW_UNIT, abs(target) * 0.30):
                    raise RuntimeError(f'{name}: wheel velocity {measured}, expected {expected}')
            if abs(odom.linear.x - linear) > 0.0015 or abs(odom.angular.z - angular) > 0.006:
                raise RuntimeError(f'{name}: odometry velocity does not match command')
            # Publish NOTHING: this measures the controller's command timeout.
            stop_seconds = self.wait_for_stop()
            delta = [end-start for start, end in zip(before, self.joint_values('position'))]
            if any(d * target <= 0 or abs(d) < 0.008 for d, target in zip(delta, expected) if abs(target) > 0.001):
                raise RuntimeError(f'{name}: encoder direction or displacement failed: {delta}')
            distance, yaw = radius * sum(delta)/2, radius * (delta[1]-delta[0])/track
            after_odom = self.state['odom']
            def yaw_of(odom):
                q = odom.pose.pose.orientation
                return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            initial_yaw = yaw_of(before_odom)
            odom_yaw = math.remainder(yaw_of(after_odom)-initial_yaw, 2*math.pi)
            dx = after_odom.pose.pose.position.x - before_odom.pose.pose.position.x
            dy = after_odom.pose.pose.position.y - before_odom.pose.pose.position.y
            odom_distance = math.cos(initial_yaw)*dx + math.sin(initial_yaw)*dy
            if abs(odom_distance-distance) > 0.001 or abs(odom_yaw-yaw) > 0.005:
                raise RuntimeError(f'{name}: odometry pose disagrees with wheel encoder changes')
            self.recorder.result(name, 'pass', commanded_linear=linear, commanded_angular=angular,
                                 input_command_scale=scale,
                                 wheel_velocity=measured, wheel_delta_rad=delta,
                                 encoder_distance_m=distance, encoder_yaw_rad=yaw,
                                 odom_distance_m=odom_distance, odom_yaw_rad=odom_yaw,
                                 command_loss_stop_seconds=stop_seconds)
        self.phase = 'stale_command'
        before = self.joint_values('position')
        self.command_for(1.0, 0.006, 0.020, stale=True)
        self.spin(0.3)
        delta = [end-start for start, end in zip(before, self.joint_values('position'))]
        if not self.check_stopped() or any(abs(d) > 0.01 for d in delta):
            raise RuntimeError('Stale timestamp command caused motion')
        self.recorder.result('stale_command', 'pass', wheel_delta_rad=delta)


def stop_process(process):
    def group_alive():
        # The launch parent can exit before a crashed controller child exits.
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                stat = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(stat[2]) == process.pid and stat[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                pass
        return False

    for sig, seconds in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        if not group_alive():
            process.wait(timeout=1)
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + seconds
        while group_alive() and time.monotonic() < deadline:
            process.poll()
            time.sleep(0.1)
    if group_alive():
        raise RuntimeError('Base process did not exit')
    process.wait(timeout=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('stationary', 'motion', 'mock'), default='stationary')
    parser.add_argument('--wheels-lifted', action='store_true')
    parser.add_argument('--domain', type=int, default=177)
    parser.add_argument('--rpm-sweep', type=float, default=0,
                        help='Explicit straight-only ramp test, up to 60 rpm')
    args = parser.parse_args()
    if args.mode == 'motion' and not args.wheels_lifted:
        parser.error('--motion mode requires --wheels-lifted')
    if not 0 <= args.domain <= 232:
        parser.error('domain must be 0..232')
    if not math.isfinite(args.rpm_sweep) or (args.rpm_sweep != 0 and not 1 <= args.rpm_sweep <= 60):
        parser.error('rpm-sweep must be 1..60 rpm')
    if args.rpm_sweep and args.mode not in ('motion', 'mock'):
        parser.error('rpm-sweep requires motion or mock mode')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    directory = ROOT / 'log/ros_base_tests' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    recorder = Recorder(directory)
    recorder.report.update(mode=args.mode, motion_requested=args.mode == 'motion', domain=args.domain,
                           rpm_sweep=args.rpm_sweep)
    print(f'ROS base test: {args.mode}, logs: {directory}', flush=True)
    settings = yaml.safe_load((PACKAGE / 'config/hardware.yaml').read_text())
    controllers = yaml.safe_load((PACKAGE / 'config/controllers.yaml').read_text())
    if args.rpm_sweep:
        base.configure_straight_speed_test(settings, controllers['diff_drive_controller']['ros__parameters'], args.rpm_sweep)
    recorder.report['settings'] = settings
    for name, content in (('hardware.yaml', settings), ('controllers.yaml', controllers)):
        (directory/name).write_text(yaml.safe_dump(content))
    originals, check, process, base_log = {}, None, None, None
    ids = [settings['left_id'], settings['right_id']]
    try:
        base.validate_settings(settings)
        raw_limit = math.floor(args.rpm_sweep/0.229) if args.rpm_sweep else 15
        base.validate_controller_limits(settings, controllers['diff_drive_controller']['ros__parameters'], raw_limit)
        if args.mode != 'mock':
            problem = port_problem(settings['motor_port'])
            if problem:
                raise RuntimeError(problem)
            base.verify_bus(settings)
            bus = MotorBus(settings)
            try:
                for motor_id in ids:
                    originals[motor_id] = {k: bus.read(motor_id, address, size) for k, address, size in (
                        ('mode', 11, 1), ('profile_acceleration', 108, 4), ('watchdog', 98, 1),
                        ('velocity_limit', 44, 4), ('drive_mode', 10, 1))}
                    recorder.event('motor_before', **bus.snapshot(motor_id), settings=originals[motor_id])
                    if originals[motor_id]['velocity_limit'] < raw_limit:
                        raise RuntimeError(f'ID {motor_id}: motor Velocity Limit is below requested speed')
                    if originals[motor_id]['drive_mode'] != 0:
                        raise RuntimeError(f'ID {motor_id}: expected Drive Mode 0')
            finally:
                bus.close()
        check = BaseCheck(args, recorder, settings)
        check.spin(1.0)
        if any(k in check.state for k in ('description', 'joints', 'odom', 'dxl')) or check.node.count_publishers('/cmd_vel') != 1:
            raise RuntimeError('Test ROS domain is already in use')
        base_log = (directory / 'base.log').open('w')
        process = subprocess.Popen(['ros2', 'launch', 'orinbot_hardware', 'base.launch.py',
                                    f'mock_hardware:={str(args.mode == "mock").lower()}',
                                    f'straight_test_rpm:={args.rpm_sweep}',
                                    f'enable_torque:={str(args.mode == "motion").lower()}'],
                                   stdout=base_log, stderr=subprocess.STDOUT, start_new_session=True)
        recorder.event('launch_start', pid=process.pid, process_group=process.pid)
        check.process = process
        check.ready()
        if args.rpm_sweep:
            check.run_rpm_sweep()
        elif args.mode in ('motion', 'mock'):
            check.run_motion()
        else:
            check.spin(3)
            if not check.check_stopped():
                raise RuntimeError('Stationary test detected wheel movement')
            recorder.result('stationary', 'pass', reason='Torque OFF; stationary ROS feedback')
        rates = {k: round((count-1)/(check.received[k]-check.first_received[k]), 2)
                 for k, count in check.counts.items() if count > 1 and check.received[k] > check.first_received[k]}
        recorder.report.update(message_counts=check.counts, message_rates_hz=rates,
                               max_wheel_velocity_rad_s=check.max_wheel_speed)
    except (Exception, KeyboardInterrupt) as exc:
        recorder.result('ros_test', 'fail', reason=str(exc) or 'Interrupted')
    finally:
        if check and process and process.poll() is None:
            try:
                check.phase, check.monitor = 'cleanup', False
                check.command_for(5.0 if args.rpm_sweep else 0.8)
            except Exception as exc:
                recorder.event('cleanup_command_error', error=str(exc))
        if process:
            try:
                stop_process(process)
                recorder.event('launch_exit', returncode=process.returncode)
            except Exception as exc:
                recorder.result('launch_shutdown', 'fail', reason=str(exc))
        if base_log:
            base_log.close()
        if check:
            check.executor.shutdown()
            check.node.destroy_node()
            check.context.shutdown()
        if process and args.mode != 'mock':
            bus = None
            try:
                problem = port_problem(settings['motor_port'])
                if problem:
                    raise RuntimeError(problem)
                bus = MotorBus(settings)
                shutdown_ok = True
                for motor_id in ids:
                    snapshot = bus.snapshot(motor_id)
                    torque, goal = bus.read(motor_id, 64), bus.read(motor_id, 104, 4)
                    recorder.event('motor_after_ros_shutdown', **snapshot, torque=torque, goal_velocity=goal)
                    shutdown_ok &= torque == 0 and goal == 0 and abs(snapshot['velocity_raw']) <= 2 and snapshot['hardware_error'] == 0
                errors = stop_motors(bus, ids, recorder)
                if not errors:
                    for motor_id in ids:
                        old = originals[motor_id]
                        if bus.read(motor_id, 11) != old['mode']:
                            bus.write(motor_id, 11, old['mode'])
                        bus.write(motor_id, 104, 0, 4)
                        bus.write(motor_id, 108, old['profile_acceleration'], 4)
                        bus.write(motor_id, 98, old['watchdog'])
                        torque, goal = bus.read(motor_id, 64), bus.read(motor_id, 104, 4)
                        recorder.event('motor_final', id=motor_id, torque=torque, goal_velocity=goal,
                                       mode=bus.read(motor_id, 11))
                        if torque != 0 or goal != 0:
                            errors.append(f'ID {motor_id}: restored state has nonzero torque/goal')
                recorder.result('motor_shutdown', 'pass' if shutdown_ok and not errors else 'fail',
                                reason='Verified torque OFF and zero goal after ROS exit' if shutdown_ok else 'ROS exit did not leave motors stopped; cleanup attempted',
                                cleanup_errors=errors)
            except Exception as exc:
                recorder.result('motor_shutdown', 'fail', reason=str(exc))
                if bus:
                    stop_motors(bus, ids, recorder)
            finally:
                if bus:
                    bus.close()
    return recorder.finish()


if __name__ == '__main__':
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    sys.exit(main())
