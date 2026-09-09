#!/usr/bin/env python3
"""Observe only the RSUSB camera; save image/IMU rates and validity checks."""
import argparse
import ctypes
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--domain', type=int, default=177)
    args = parser.parse_args()
    if not 10 <= args.duration <= 120 or not 0 <= args.domain <= 232:
        parser.error('duration must be 10..120 seconds and domain 0..232')
    ctypes.CDLL('librealsense2.so.2.58')
    sdk_paths = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if '/librealsense2.so.' in line})
    if not sdk_paths or any('/librealsense-2.58.1-rsusb/' not in p for p in sdk_paths):
        parser.error('Run this script using tools/with_rsusb.sh')
    if subprocess.run(['pgrep', '-f', '[r]ealsense2_camera_node|[r]ealsense-viewer'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        parser.error('Another camera driver is running; stop it before this test')

    workspace = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'interface_checks', workspace / 'src/orinbot_hardware/scripts/test_interfaces.py')
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu

    directory = workspace / 'log/rsusb_camera' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    print(f'로그 폴더: {directory}', flush=True)
    recorder = checks.Recorder(directory)
    recorder.report.update(sdk_paths=sdk_paths, duration_seconds=args.duration, domain=args.domain)
    rclpy.init(domain_id=args.domain)
    node = rclpy.create_node('rsusb_camera_check')
    specs = [('color', '/camera/color/image_raw', Image, 10),
             ('depth', '/camera/aligned_depth_to_color/image_raw', Image, 10),
             ('imu', '/camera/imu', Imu, 20)]
    stats = {topic: dict(messages=0, first=None, last=None, previous_stamp=None,
                         nonincreasing_stamps=0, max_gap=0.0, valid_samples=0,
                         invalid_samples=0, last_sample=0.0) for _, topic, _, _ in specs}

    def callback(kind, topic):
        def receive(message):
            now = time.monotonic()
            s = stats[topic]
            s['messages'] += 1
            if s['first'] is None:
                s['first'] = now
            if s['last'] is not None:
                s['max_gap'] = max(s['max_gap'], now - s['last'])
            s['last'] = now
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if s['previous_stamp'] is not None and stamp <= s['previous_stamp']:
                s['nonincreasing_stamps'] += 1
            s['previous_stamp'] = stamp
            if now - s['last_sample'] >= 1:
                s['last_sample'] = now
                try:
                    sample = checks.sensor_summary(kind, message)
                except Exception as exc:
                    sample = {'valid': False, 'error': str(exc)}
                s['valid_samples' if sample['valid'] else 'invalid_samples'] += 1
                recorder.event('sensor_sample', topic=topic, stamp=stamp, **sample)
        return receive

    subscriptions = [node.create_subscription(msgtype, topic, callback(kind, topic),
                                               qos_profile_sensor_data)
                     for kind, topic, msgtype, _ in specs]
    process = None
    driver_log = (directory / 'camera.log').open('w')
    try:
        command = ['ros2', 'launch', 'orinbot_hardware', 'sensors.launch.py', 'lidar:=false']
        process = subprocess.Popen(command, stdout=driver_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        recorder.event('driver_started', command=command, pid=process.pid)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        topics = {}
        passed = process.poll() is None
        for _, topic, _, minimum_hz in specs:
            s = stats[topic]
            elapsed = (s['last'] - s['first']) if s['messages'] >= 2 else 0
            hz = (s['messages'] - 1) / elapsed if elapsed else 0
            fresh = s['last'] is not None and time.monotonic() - s['last'] < 3
            good = (hz >= minimum_hz and elapsed >= args.duration / 2 and fresh
                    and s['valid_samples'] >= 2 and s['invalid_samples'] == 0
                    and s['nonincreasing_stamps'] == 0 and s['max_gap'] < 1)
            topics[topic] = {'messages': s['messages'], 'hz': round(hz, 2),
                             'observed_seconds': round(elapsed, 2), 'fresh': fresh,
                             'max_gap_seconds': round(s['max_gap'], 3),
                             'nonincreasing_stamps': s['nonincreasing_stamps'],
                             'valid_samples': s['valid_samples'],
                             'invalid_samples': s['invalid_samples'], 'pass': good}
            passed &= good
        recorder.result('camera', 'pass' if passed else 'fail', topics=topics,
                        reason='RSUSB 컬러·깊이·IMU 수신률·내용·시간 검사'
                        + (' 통과' if passed else ' 실패; camera.log 확인'))
    except (KeyboardInterrupt, Exception) as exc:
        recorder.result('camera', 'fail', reason=f'{type(exc).__name__}: {exc}')
    finally:
        if process is not None:
            for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
                try:
                    os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
                except ProcessLookupError:
                    break
        driver_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return recorder.finish()


if __name__ == '__main__':
    raise SystemExit(main())
