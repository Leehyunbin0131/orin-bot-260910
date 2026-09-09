#!/usr/bin/env python3
"""Test the connected lidar, camera and XM430 motors; save a timestamped report.

Default: sensor streams and read-only motor checks. --motion --wheels-lifted
adds brief, low-speed wheel tests. Run with other robot applications stopped.
"""
import argparse
from datetime import datetime
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


def signed32(value):
    return value - 2**32 if value >= 2**31 else value


def finite(values):
    return all(math.isfinite(v) for v in values)


class Recorder:
    def __init__(self, directory):
        self.directory = directory
        self.events = (directory / 'events.jsonl').open('w')
        self.report = {'started_at': datetime.now().astimezone().isoformat(),
                       'interfaces': {}, 'motion_requested': False}

    def event(self, kind, **data):
        record = {'time': datetime.now().astimezone().isoformat(),
                  'monotonic': time.monotonic(), 'kind': kind, **data}
        self.events.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        self.events.flush()

    def result(self, name, status, **data):
        self.report['interfaces'][name] = {'status': status, **data}
        self.event('result', interface=name, status=status, **data)
        print(f'[{status.upper()}] {name}: {data.get("reason", "완료")}', flush=True)

    def finish(self):
        self.report['finished_at'] = datetime.now().astimezone().isoformat()
        results = self.report['interfaces'].values()
        self.report['status'] = 'pass' if results and all(r['status'] == 'pass' for r in results) else 'fail'
        (self.directory / 'summary.json').write_text(
            json.dumps(self.report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        lines = [f"결과: {self.report['status'].upper()}",
                 f"시작: {self.report['started_at']}",
                 f"모터 회전 시험 요청: {self.report['motion_requested']}", '']
        for name, result in self.report['interfaces'].items():
            lines.append(f"{name}: {result['status'].upper()} — {result.get('reason', '완료')}")
        lines.extend(['', 'summary.json: 전체 결과 / events.jsonl: 센서 요약·모터 측정·오류',
                      'camera.log, lidar.log: 각 드라이버 출력',
                      'PASS는 센서 수신과 엔코더 응답 기준이며 바닥 주행 방향·SLAM 정확도 판정은 아닙니다.'])
        (self.directory / 'summary.txt').write_text('\n'.join(lines) + '\n')
        self.events.close()
        print(f'로그 저장: {self.directory}', flush=True)
        return 0 if self.report['status'] == 'pass' else 1


def port_problem(path):
    if not Path(path).exists():
        return f'장치 없음: {path}'
    if not os.access(path, os.R_OK | os.W_OK):
        return f'시리얼 접근 권한 없음: {path}; prepare_host.sh 실행 필요'
    # Reject a port already held by another process belonging to this user.
    target = os.path.realpath(path)
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            for fd in (proc / 'fd').iterdir():
                try:
                    if os.readlink(fd) == target:
                        return f'장치 사용 중: {target}, PID={proc.name}; 기존 제어 프로그램 종료 필요'
                except OSError:
                    pass
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            pass
    return None


class MotorBus:
    def __init__(self, settings):
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
        self.success = COMM_SUCCESS
        self.port = PortHandler(settings['motor_port'])
        self.packet = PacketHandler(2.0)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(settings['baud_rate']):
                raise RuntimeError('모터 포트 또는 통신 속도 설정 실패')
            # Linux pyserial excludes later opens while this test owns the port.
            self.port.ser.exclusive = True
        except BaseException:
            self.port.closePort()
            raise

    def check(self, result, error, label):
        if result != self.success or error:
            raise RuntimeError(f'{label}: {self.packet.getTxRxResult(result)} / '
                               f'{self.packet.getRxPacketError(error)}')

    def ping(self, motor_id):
        model, result, error = self.packet.ping(self.port, motor_id)
        self.check(result, error, f'ID {motor_id} PING')
        if model != 1030:
            raise RuntimeError(f'ID {motor_id}: XM430-W210 모델 아님 ({model})')
        return model

    def read(self, motor_id, address, size=1):
        value, result, error = getattr(self.packet, f'read{size}ByteTxRx')(
            self.port, motor_id, address)
        self.check(result, error, f'ID {motor_id} READ {address}')
        return value

    def write(self, motor_id, address, value, size=1):
        result, error = getattr(self.packet, f'write{size}ByteTxRx')(
            self.port, motor_id, address, value & ((1 << (size * 8)) - 1))
        self.check(result, error, f'ID {motor_id} WRITE {address}')

    def snapshot(self, motor_id):
        started = time.monotonic()
        block, result, error = self.packet.readTxRx(self.port, motor_id, 126, 21)
        self.check(result, error, f'ID {motor_id} telemetry')
        if len(block) != 21:
            raise RuntimeError(f'ID {motor_id}: 불완전한 엔코더 응답')
        data = bytes(block)
        raw = lambda offset, size, signed=False: int.from_bytes(
            data[offset:offset + size], 'little', signed=signed)
        sample = {'id': motor_id, 'current_raw': raw(0, 2, True),
                  'velocity_raw': raw(2, 4, True), 'position_raw': raw(6, 4, True),
                  'voltage': raw(18, 2) * 0.1, 'temperature_c': raw(20, 1),
                  'hardware_error': self.read(motor_id, 70)}
        sample['read_ms'] = round((time.monotonic() - started) * 1000, 2)
        return sample

    def close(self):
        self.port.closePort()


def stop_motors(bus, ids, recorder):
    """Try every motor even when stopping one fails. Torque OFF is first."""
    failures = []
    for motor_id in ids:
        for address, value, size in ((64, 0, 1), (98, 0, 1), (104, 0, 4)):
            try:
                bus.write(motor_id, address, value, size)
            except Exception as exc:
                failures.append(str(exc))
    for motor_id in ids:
        try:
            if bus.read(motor_id, 64) != 0 or bus.read(motor_id, 104, 4) != 0:
                failures.append(f'ID {motor_id}: 토크 OFF / 목표 속도 0 확인 실패')
        except Exception as exc:
            failures.append(str(exc))
    recorder.event('motor_stop', verified=not failures, errors=failures)
    return failures


def motor_test(settings, args, recorder):
    bus = None
    changed = []
    originals = {}
    phases = []
    outcome = 'pass'
    reason = '57600bps 모터 통신·정지 상태 조회 완료; 회전 시험 미요청'
    try:
        problem = port_problem(settings['motor_port'])
        if problem:
            raise RuntimeError(problem)
        if importlib.util.find_spec('dynamixel_sdk') is None:
            raise RuntimeError('dynamixel_sdk 미설치; prepare_host.sh 실행 필요')
        ids = [settings['left_id'], settings['right_id']]
        if len(set(ids)) != 2 or any(type(i) is not int or not 0 <= i <= 252 for i in ids):
            raise ValueError('서로 다른 정상 모터 ID 두 개가 필요합니다')
        if any(settings[f'{s}_direction'] not in (-1, 1) for s in ('left', 'right')):
            raise ValueError('모터 방향값은 1 또는 -1이어야 합니다')
        bus = MotorBus(settings)
        for motor_id in ids:
            bus.ping(motor_id)
            registers = {name: bus.read(motor_id, address, size) for name, address, size in (
                ('firmware', 6, 1), ('drive_mode', 10, 1), ('mode', 11, 1),
                ('torque', 64, 1), ('return_level', 68, 1), ('watchdog', 98, 1),
                ('profile_acceleration', 108, 4))}
            originals[motor_id] = registers
            sample = bus.snapshot(motor_id)
            recorder.event('motor_initial', **sample, registers=registers)
            if registers['torque'] or sample['hardware_error'] or sample['velocity_raw']:
                raise RuntimeError(f'ID {motor_id}: 토크 ON, 하드웨어 오류 또는 회전 중; 시험 거부')
            if args.motion and (registers['firmware'] < 38 or registers['return_level'] != 2
                                or registers['watchdog'] == 255):
                raise RuntimeError(f'ID {motor_id}: 펌웨어/응답 레벨/워치독 점검 필요')
        if args.motion:
            # Preflight of BOTH motors has passed before the first write.
            for motor_id in ids:
                changed.append(motor_id)
                if originals[motor_id]['mode'] != 1:
                    bus.write(motor_id, 11, 1)
                bus.write(motor_id, 104, 0, 4)
                bus.write(motor_id, 108, 5, 4)
                bus.write(motor_id, 98, 15)
            for motor_id in ids:
                bus.write(motor_id, 64, 1)
            left = settings['left_direction'] * args.velocity_raw
            right = settings['right_direction'] * args.velocity_raw
            for name, goals in (
                    ('left_wheel', [left, 0]), ('right_wheel', [0, right]),
                    ('forward', [left, right]), ('reverse', [-left, -right])):
                before = [bus.snapshot(i)['position_raw'] for i in ids]
                start = time.monotonic()
                while time.monotonic() - start < args.motion_seconds:
                    tick = time.monotonic()
                    for motor_id, velocity in zip(ids, goals):
                        bus.write(motor_id, 104, velocity, 4)
                    for motor_id in ids:
                        sample = bus.snapshot(motor_id)
                        recorder.event('motor_sample', phase=name, **sample)
                        if sample['hardware_error']:
                            raise RuntimeError(f'ID {motor_id}: 주행 중 하드웨어 오류')
                    time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
                for motor_id in ids:
                    bus.write(motor_id, 104, 0, 4)
                settled = time.monotonic() + 0.5
                while time.monotonic() < settled:
                    for motor_id in ids:
                        recorder.event('motor_sample', phase=name + '_stopping', **bus.snapshot(motor_id))
                    time.sleep(0.05)
                after = [bus.snapshot(i) for i in ids]
                deltas = [signed32((s['position_raw'] - p) & 0xffffffff)
                          for s, p in zip(after, before)]
                passed = all((g == 0 and abs(d) < 15) or (g * d > 0 and abs(d) >= 5)
                             for g, d in zip(goals, deltas))
                passed &= all(abs(s['velocity_raw']) <= 2 for s in after)
                phases.append({'phase': name, 'goals_raw': goals,
                               'encoder_delta': deltas, 'pass': passed})
                recorder.event('motor_phase', **phases[-1])
                if not passed:
                    raise RuntimeError(f'{name}: 엔코더 방향·변화량 또는 정지 검사 실패')
            reason = '좌륜·우륜·양륜 정/역회전 엔코더 응답 및 정지 검사 통과'
        else:
            reason = f"{settings['baud_rate']}bps 모터 통신 조회 완료; 회전 시험 미요청"
    except Exception as exc:
        outcome, reason = 'fail', str(exc)
        recorder.event('motor_error', error=reason)
    finally:
        if bus:
            if changed:
                errors = stop_motors(bus, changed, recorder)
                if not errors:
                    for motor_id in changed:
                        try:
                            original = originals[motor_id]
                            if original['mode'] != 1:
                                bus.write(motor_id, 11, original['mode'])
                            # Operating Mode resets Goal Velocity (observed: 330).
                            bus.write(motor_id, 104, 0, 4)
                            bus.write(motor_id, 108, original['profile_acceleration'], 4)
                            bus.write(motor_id, 98, original['watchdog'])
                            if bus.read(motor_id, 64) != 0 or bus.read(motor_id, 104, 4) != 0:
                                raise RuntimeError(f'ID {motor_id}: 복구 후 토크/목표 속도 확인 실패')
                        except Exception as exc:
                            errors.append(str(exc))
                if errors:
                    outcome, reason = 'fail', '정지/설정 복구 확인 실패: ' + '; '.join(errors)
            bus.close()
    recorder.result('motors', outcome, reason=reason, phases=phases,
                    motion_tested=bool(phases), initial_settings=originals)


def sensor_summary(kind, message):
    if kind == 'scan':
        valid = [v for v in message.ranges if math.isfinite(v)
                 and message.range_min <= v <= message.range_max]
        return {'valid': bool(valid), 'rays': len(message.ranges), 'valid_rays': len(valid),
                'min_m': min(valid) if valid else None, 'max_m': max(valid) if valid else None}
    if kind == 'imu':
        gyro = [getattr(message.angular_velocity, k) for k in 'xyz']
        accel = [getattr(message.linear_acceleration, k) for k in 'xyz']
        good = finite(gyro + accel)
        return {'valid': good, 'gyro': gyro if good else None, 'accel': accel if good else None}
    if kind == 'points':
        return {'valid': message.width * message.height > 0 and bool(message.data),
                'points': message.width * message.height}
    result = {'valid': message.width > 0 and message.height > 0
              and len(message.data) == message.step * message.height,
              'width': message.width, 'height': message.height, 'encoding': message.encoding}
    if kind == 'depth' and result['valid']:
        import numpy as np
        if message.encoding not in ('16UC1', '32FC1'):
            result.update(valid=False, error='Unsupported depth encoding')
            return result
        dtype = np.dtype(('>' if message.is_bigendian else '<') +
                         ('u2' if message.encoding == '16UC1' else 'f4'))
        data = np.ndarray((message.height, message.width), dtype=dtype,
                          buffer=bytes(message.data), strides=(message.step, dtype.itemsize))
        sampled = data[::8, ::8].astype(float).ravel()
        valid = sampled[np.isfinite(sampled) & (sampled > 0)]
        meters = valid * (0.001 if message.encoding == '16UC1' else 1.0)
        result.update(valid=bool(valid.size), valid_fraction=float(valid.size / sampled.size),
                      min_m=float(meters.min()) if meters.size else None,
                      median_m=float(np.median(meters)) if meters.size else None,
                      max_m=float(meters.max()) if meters.size else None)
    return result


def sensor_test(settings, args, recorder):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    processes = {}
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=args.domain)
    node = rclpy.create_node('orinbot_interface_test', context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    stats = {}
    subscriptions = []
    specs = {
        'lidar': [('scan', '/scan', LaserScan, True)],
        'camera': [('color', '/camera/color/image_raw', Image, True),
                   ('depth', '/camera/aligned_depth_to_color/image_raw', Image, True),
                   ('imu', '/camera/imu', Imu, True),
                   ('points', '/camera/depth/color/points', PointCloud2, False)],
    }

    def callback(kind, topic):
        def receive(message):
            now = time.monotonic()
            s = stats[topic]
            s['messages'] += 1
            if s['first'] is None:
                s['first'] = now
            s['last'] = now
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if s['previous_stamp'] is not None and stamp <= s['previous_stamp']:
                s['nonincreasing_stamps'] += 1
            s['previous_stamp'] = stamp
            if now - s['last_logged'] >= 1:
                try:
                    sample = sensor_summary(kind, message)
                except Exception as exc:
                    sample = {'valid': False, 'error': str(exc)}
                s['valid_samples'] += int(sample['valid'])
                s['last_logged'] = now
                recorder.event('sensor_sample', topic=topic, frame=message.header.frame_id,
                               stamp=stamp, **sample)
        return receive

    try:
        for name, package in (('lidar', 'sllidar_ros2'), ('camera', 'realsense2_camera')):
            problems = []
            try:
                get_package_prefix(package)
            except PackageNotFoundError:
                problems.append(f'{package} 미설치')
            if name == 'lidar':
                problem = port_problem(settings['lidar_port'])
                if problem:
                    problems.append(problem)
            if problems:
                recorder.result(name, 'fail', reason='; '.join(problems))
                continue
            for kind, topic, msgtype, required in specs[name]:
                stats[topic] = {'messages': 0, 'valid_samples': 0, 'first': None, 'last': None,
                                'last_logged': 0, 'nonincreasing_stamps': 0, 'previous_stamp': None}
                subscriptions.append(node.create_subscription(
                    msgtype, topic, callback(kind, topic), qos_profile_sensor_data))
            log = (recorder.directory / f'{name}.log').open('w')
            command = ['ros2', 'launch', 'orinbot_hardware', 'sensors.launch.py',
                       f'config:={args.config}', f'lidar:={str(name == "lidar").lower()}',
                       f'camera:={str(name == "camera").lower()}']
            env = {**os.environ, 'ROS_DOMAIN_ID': str(args.domain),
                   'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
            try:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           env=env, start_new_session=True)
            except Exception:
                log.close()
                raise
            processes[name] = (process, log)
            recorder.event('driver_started', interface=name, pid=process.pid, command=command)
        if processes:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            for name, (process, _) in processes.items():
                topics = {}
                passed = process.poll() is None
                for _, topic, _, required in specs[name]:
                    s = stats[topic]
                    elapsed = (s['last'] - s['first']) if s['messages'] >= 2 else 0
                    fresh = s['last'] is not None and time.monotonic() - s['last'] < 3.0
                    good = s['messages'] >= 3 and s['valid_samples'] >= 2 and fresh
                    good &= s['nonincreasing_stamps'] == 0
                    topics[topic] = {'messages': s['messages'],
                                     'hz': round((s['messages'] - 1) / elapsed, 2) if elapsed else 0,
                                     'valid_samples': s['valid_samples'], 'fresh': fresh,
                                     'nonincreasing_stamps': s['nonincreasing_stamps'],
                                     'required': required, 'pass': good}
                    if required:
                        passed &= good
                recorder.result(name, 'pass' if passed else 'fail', topics=topics,
                                reason='필수 스트림 수신·내용·시간 검사 통과' if passed
                                else f'스트림 누락/오류/중단; {name}.log 및 summary.json 확인')
    finally:
        for name, (process, log) in processes.items():
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            except ProcessLookupError:
                pass
            finally:
                log.close()
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            context.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, default=Path.home() / 'ros2_ws/log/interface_tests')
    parser.add_argument('--duration', type=float, default=30, help='Sensor observation seconds (10-120)')
    parser.add_argument('--domain', type=int, default=11, help='ROS domain (robot default: 11)')
    parser.add_argument('--motion', action='store_true', help='Perform brief motor rotation tests')
    parser.add_argument('--wheels-lifted', action='store_true', help='Confirm both drive wheels are lifted and secured')
    parser.add_argument('--velocity-raw', type=int, default=15, help='Dynamixel velocity units (1-15; verified low-speed range)')
    parser.add_argument('--motion-seconds', type=float, default=1.0, help='Each rotation phase (0.5-2 seconds)')
    args = parser.parse_args()
    if args.motion and not args.wheels_lifted:
        parser.error('--motion에는 바퀴를 띄워 고정한 뒤 --wheels-lifted가 필요합니다')
    if not 10 <= args.duration <= 120 or not 0.5 <= args.motion_seconds <= 2:
        parser.error('duration은 10~120초, motion-seconds는 0.5~2초 범위입니다')
    if not 1 <= args.velocity_raw <= 15 or not 0 <= args.domain <= 232:
        parser.error('velocity-raw는 1~15, domain은 0~232 범위입니다')
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = Path(get_package_share_directory('orinbot_hardware')) / 'config/hardware.yaml'
    args.config = args.config.resolve()
    return args


def main():
    args = parse_args()
    directory = args.output.expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    recorder = Recorder(directory)
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    recorder.report['motion_requested'] = args.motion
    print(f'로그 폴더: {directory}', flush=True)
    try:
        settings = yaml.safe_load(args.config.read_text())
        recorder.report['settings'] = settings
        recorder.report['test_parameters'] = {'duration': args.duration, 'domain': args.domain,
                                             'velocity_raw': args.velocity_raw,
                                             'motion_seconds': args.motion_seconds}
        recorder.report['host'] = {
            'uid': os.getuid(), 'groups': os.getgroups(),
            'serial_ports': {key: {'path': settings[key],
                                   'exists': Path(settings[key]).exists(),
                                   'accessible': os.access(settings[key], os.R_OK | os.W_OK)}
                             for key in ('motor_port', 'lidar_port')},
        }
        for filename, command in (('usb.txt', ['lsusb']), ('usb_tree.txt', ['lsusb', '-t'])):
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=5)
                (directory / filename).write_text(result.stdout + result.stderr)
            except (OSError, subprocess.TimeoutExpired) as exc:
                recorder.event('host_inventory_error', command=command, error=str(exc))
        (directory / 'hardware.yaml').write_text(yaml.safe_dump(settings, allow_unicode=True))
        recorder.event('test_started', motion=args.motion)
        # Sensor and motor phases are separate; one failed interface does not hide the others.
        try:
            sensor_test(settings, args, recorder)
        except Exception as exc:
            recorder.event('sensor_error', error=str(exc))
            for name in ('lidar', 'camera'):
                if name not in recorder.report['interfaces']:
                    recorder.result(name, 'fail', reason=str(exc))
        motor_test(settings, args, recorder)
    except KeyboardInterrupt:
        recorder.result('interrupted', 'fail', reason='사용자 중단; 모터 정지·드라이버 종료 처리 수행')
    except Exception as exc:
        recorder.result('test_runner', 'fail', reason=str(exc))
    return recorder.finish()


if __name__ == '__main__':
    sys.exit(main())
