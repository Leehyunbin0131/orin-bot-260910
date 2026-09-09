#!/usr/bin/env python3
"""Follow the rectangle using EKF odometry; stop if any contributing sensor fails."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rtabmap_msgs.msg import OdomInfo
from ament_index_python.packages import get_package_share_directory

from drive_rectangle import Rectangle, STOP_PUBLISH_SECONDS, wrap

SOURCES = {
    'wheel': (Odometry, '/odom', 0.5),
    'lidar': (Odometry, '/lidar/odom', 1.0),
    'imu': (Imu, '/camera/imu', 0.5),
    'scan': (LaserScan, '/scan', 1.0),
    'tracking': (OdomInfo, '/lidar/odom_info', 1.0),
    'filtered': (Odometry, '/odometry/filtered', 0.5),
}


def sample(kind, message):
    if not message.header.frame_id:
        raise ValueError('Empty sensor frame')
    if kind in ('wheel', 'lidar', 'filtered'):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        t = message.twist.twist
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, t.linear.x, t.angular.z]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite odometry')
        if not 0.99 <= sum(v*v for v in (q.x, q.y, q.z, q.w)) <= 1.01:
            raise ValueError('Invalid odometry quaternion')
        if (message.header.frame_id, message.child_frame_id) != ('odom', 'base_footprint'):
            raise ValueError('Unexpected odometry frames')
        covariance = [message.pose.covariance[i] for i in (0, 7, 35)]
        if not all(math.isfinite(v) and 0 <= v <= 0.25 for v in covariance):
            raise ValueError('Odometry covariance is invalid or too large')
        return {'pose': [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
                'velocity': [t.linear.x, t.angular.z], 'covariance_xy_yaw': covariance}
    if kind == 'imu':
        gyro = [getattr(message.angular_velocity, a) for a in 'xyz']
        covariance = [message.angular_velocity_covariance[i] for i in (0, 4, 8)]
        if not all(math.isfinite(v) for v in gyro) or not all(
                math.isfinite(v) and v >= 0 for v in covariance):
            raise ValueError('IMU angular velocity unavailable or invalid')
        return {'gyro': gyro, 'gyro_covariance': covariance}
    if kind == 'scan':
        valid = sum(math.isfinite(v) and message.range_min <= v <= message.range_max
                    for v in message.ranges)
        if valid < 30:
            raise ValueError('Too few usable laser ranges')
        return {'valid_ranges': valid}
    if message.lost or not math.isfinite(message.icp_inliers_ratio) or message.icp_inliers_ratio < 0.3:
        raise ValueError('LiDAR tracking lost or insufficient scan correspondences')
    return {'inlier_ratio': message.icp_inliers_ratio,
            'estimation_seconds': message.time_estimation}


class InputHealth:
    """Fresh output alone is insufficient: EKF can predict after its sensors stop."""
    def __init__(self):
        self.states = {name: dict(count=0, first=None, arrival=None, stamp=None,
                                 valid=False, error='Waiting for input', data=None)
                       for name in SOURCES}
        self.armed = False
        self.fault = None

    def update(self, kind, message, now, ros_now):
        state = self.states[kind]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        try:
            if not 0 < stamp or not -0.2 <= ros_now-stamp <= SOURCES[kind][2]:
                raise ValueError('Sensor timestamp is stale or in the future')
            if state['stamp'] is not None and stamp <= state['stamp']:
                raise ValueError('Sensor timestamp repeated or moved backwards')
            data = sample(kind, message)
            if self.armed and kind in ('lidar', 'filtered') and state['data']:
                previous = state['data']['pose']
                current = data['pose']
                elapsed = stamp-state['stamp']
                if (math.hypot(current[0]-previous[0], current[1]-previous[1]) > 0.10+0.3*elapsed
                        or abs(wrap(current[2]-previous[2])) > 0.20+0.5*elapsed):
                    raise ValueError('Odometry jumped or reset during the test')
            state.update(valid=True, error=None, data=data)
            state['count'] += 1
            state['first'] = now if state['first'] is None else state['first']
        except ValueError as exc:
            state.update(valid=False, error=str(exc))
            if self.armed:
                self.fault = self.fault or f'{kind}: {exc}'
        state.update(arrival=now, stamp=stamp)

    def problem(self, now, ros_now):
        if self.fault:
            return self.fault
        for name, state in self.states.items():
            if not state['valid'] or state['count'] < 5:
                return f'{name}: {state["error"] or "Waiting for five valid samples"}'
            if now-state['arrival'] > SOURCES[name][2] or ros_now-state['stamp'] > SOURCES[name][2]:
                return f'{name}: Sensor input stopped'
        return None


class MotionAgreement:
    """Compare segment-relative motion; estimator odom origins may differ."""
    def __init__(self, states, turning=False):
        self.origins = {k: tuple(states[k]['data']['pose'])
                        for k in ('wheel', 'lidar', 'filtered')}
        self.turning = turning

    def measure(self, states):
        result = {}
        for kind, (x0, y0, yaw0) in self.origins.items():
            x, y, yaw = states[kind]['data']['pose']
            result[kind] = {
                'forward_m': math.cos(yaw0)*(x-x0)+math.sin(yaw0)*(y-y0),
                'yaw_rad': wrap(yaw-yaw0),
            }
        return result

    def problem(self, motion):
        field = 'yaw_rad' if self.turning else 'forward_m'
        wheel, lidar = (motion[k][field] for k in ('wheel', 'lidar'))
        # Ignore small scan noise and startup latency. Never auto-flip sensor data.
        wheel_min, lidar_min = (0.10, 0.07) if self.turning else (0.03, 0.02)
        if abs(wheel) >= wheel_min and abs(lidar) >= lidar_min and wheel*lidar < 0:
            unit = 'rad' if self.turning else 'm'
            return (f'Wheel/LiDAR direction mismatch: wheel={wheel:+.3f}{unit}, '
                    f'lidar={lidar:+.3f}{unit}; check sensor mounting and wheel directions')
        return None


class FusedRectangle(Rectangle):
    def __init__(self, args):
        args.odom_topic = '/odometry/filtered'
        args.log_subdir = 'rectangle_fused_runs'
        args.mock = False
        super().__init__(args)
        self.health = InputHealth()
        self.source_subs = [self.create_subscription(msgtype, topic,
            lambda message, key=name: self.health.update(
                key, message, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9),
            qos_profile_sensor_data) for name, (msgtype, topic, _) in SOURCES.items()]
        self.last_source_log = 0
        self.motion_check = None
        self.last_motion_log = 0
        self.summary.update(odometry_topic=args.odom_topic, mode='observe' if args.observe else 'drive',
                            sources={k: v[1] for k, v in SOURCES.items()},
                            fusion='robot_localization EKF: wheel vx/wz + ICP x/y/yaw + IMU wz',
                            lidar_yaw_degrees=float(yaml.safe_load(args.config.read_text())[
                                'lidar_yaw_degrees']))

    def health_problem(self):
        return self.health.problem(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)

    def wait_ready(self):
        deadline = time.monotonic()+45
        ready_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.health_problem() is None:
                ready_since = ready_since or time.monotonic()
                if time.monotonic()-ready_since >= 2:
                    break
            else:
                ready_since = None
        else:
            raise RuntimeError('Sensor fusion not ready: ' + str(self.health_problem()))
        self.health.armed = True
        super().wait_ready()
        torque = [p.text for p in ET.fromstring(self.description).findall(
            "./ros2_control/gpio/param[@name='Torque Enable']")]
        if torque != [str(int(not self.args.observe))]*2:
            raise RuntimeError('Unexpected torque configuration for observation/drive mode')
        self.summary['inputs_ready'] = True

    def tick(self, linear=0.0, angular=0.0):
        problem = self.health_problem()
        if problem:
            raise RuntimeError(problem)
        if self.args.observe and (linear or angular):
            raise RuntimeError('Motion commands are disabled in observation mode')
        if self.motion_check is not None:
            motion = self.motion_check.measure(self.health.states)
            self.summary['last_motion'] = {'phase': self.phase, **motion}
            problem = self.motion_check.problem(motion)
            now = time.monotonic()
            if problem or now-self.last_motion_log >= 1.0:
                self.record('motion_progress', **motion)
                self.last_motion_log = now
                field = 'yaw_rad' if self.motion_check.turning else 'forward_m'
                scale = 180/math.pi if self.motion_check.turning else 1
                unit = '도' if self.motion_check.turning else 'm'
                values = ', '.join(f'{label} {motion[k][field]*scale:+.3f}{unit}'
                                   for k, label in (('wheel', '엔코더'), ('lidar', '라이다'),
                                                    ('filtered', '융합')))
                print(f'{self.phase}: {values}', flush=True)
            if problem:
                raise RuntimeError(problem)
        super().tick(linear, angular)
        now = time.monotonic()
        if now-self.last_source_log >= 0.2:
            self.record('fusion_inputs', **{k: s['data'] for k, s in self.health.states.items()})
            self.last_source_log = now

    def segment(self, length, heading, turning=False):
        self.motion_check = MotionAgreement(self.health.states, turning)
        self.last_motion_log = 0
        try:
            return super().segment(length, heading, turning)
        finally:
            self.motion_check = None

    def run(self):
        if not self.args.observe:
            return super().run()
        self.wait_ready()
        self.phase = 'observing'
        start, deadline = self.pose, time.monotonic()+self.args.duration
        drift = 0.0
        while time.monotonic() < deadline:
            self.tick()
            drift = max(drift, math.hypot(self.pose[0]-start[0], self.pose[1]-start[1]))
        self.summary.update(status='pass', start_pose=start, final_pose=self.pose,
                            observed_seconds=self.args.duration, max_displacement_m=drift)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=float, default=1.0)
    parser.add_argument('--height', type=float, default=1.0)
    parser.add_argument('--rpm', type=float, default=60)
    parser.add_argument('--turn-rpm', type=float, default=25)
    parser.add_argument('--observe', action='store_true')
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--config', type=Path, default=Path(get_package_share_directory(
        'orinbot_hardware')) / 'config/hardware.yaml')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) and 0.1 <= v <= 5 for v in (args.width, args.height)):
        parser.error('Side lengths must be 0.1..5 metres')
    if not math.isfinite(args.rpm) or not 1 <= args.rpm <= 60:
        parser.error('Wheel rpm must be 1..60')
    if not math.isfinite(args.turn_rpm) or not 1 <= args.turn_rpm <= 25:
        parser.error('Turn wheel rpm must be 1..25 (also capped by --rpm)')
    if not math.isfinite(args.duration) or not 10 <= args.duration <= 120:
        parser.error('Duration must be 10..120 seconds')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FusedRectangle(args)
    print(f'로그: {node.directory}', flush=True)
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        node.summary.update(status='fail', reason=str(exc) or 'Interrupted')
        print(f'중단: {node.summary["reason"]}', flush=True)
    finally:
        node.phase = 'stopping'
        deadline = time.monotonic()+STOP_PUBLISH_SECONDS
        while time.monotonic() < deadline and rclpy.ok():
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
        # Check the raw encoders even if filtered odometry disappeared during failure.
        wheel = node.health.states['wheel']
        stopped = (wheel['valid'] and time.monotonic()-wheel['arrival'] < 0.5
                   and abs(wheel['data']['velocity'][0]) < 0.003
                   and abs(wheel['data']['velocity'][1]) < 0.01)
        node.summary['stopped'] = stopped
        if not stopped:
            node.summary.update(status='fail', reason='Final encoder stop feedback not confirmed')
        node.summary['input_counts'] = {k: s['count'] for k, s in node.health.states.items()}
        (node.directory / 'summary.json').write_text(json.dumps(node.summary, indent=2)+'\n')
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
