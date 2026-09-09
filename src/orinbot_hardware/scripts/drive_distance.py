#!/usr/bin/env python3
"""Drive both XM430 wheels forward for a bounded encoder distance, then stop."""
import argparse
from datetime import datetime
import math
from pathlib import Path
import signal
import sys
import time

import yaml

from test_interfaces import MotorBus, Recorder, port_problem, signed32, stop_motors

VELOCITY_UNIT = 0.0239691227  # rad/s per XM430 Goal Velocity unit
TICKS_PER_TURN = 4096


def wheel_distances(before, after, radius, directions):
    meters_per_tick = 2 * math.pi * radius / TICKS_PER_TURN
    return [signed32((end - start) & 0xffffffff) * direction * meters_per_tick
            for start, end, direction in zip(before, after, directions)]


def goals_for_distance(distances, remaining, speed):
    # Reduce the leading wheel; never exceed the user-confirmed low speed.
    base = min(speed, 6) if remaining < 0.02 else speed
    error = distances[0] - distances[1]
    correction = min(max(0, base - 4), int(abs(error) / 0.001))
    return [base - correction, base] if error > 0 else [base, base - correction]


def drive(settings, args, recorder):
    bus = None
    changed = []
    originals = {}
    distances = [0.0, 0.0]
    success = False
    reason = ''
    baseline = None
    ids = [settings['left_id'], settings['right_id']]
    directions = [settings['left_direction'], settings['right_direction']]
    radius = float(settings['wheel_radius'])
    recorder.report['motion_requested'] = True
    recorder.report['distance_target_m'] = args.distance
    recorder.report['placement'] = args.placement
    recorder.report['speed_limit_raw'] = args.velocity_raw
    try:
        if len(set(ids)) != 2 or any(type(i) is not int or not 0 <= i <= 252 for i in ids):
            raise ValueError('서로 다른 정상 모터 ID 두 개가 필요합니다')
        if not math.isfinite(radius) or radius <= 0 or any(d not in (-1, 1) for d in directions):
            raise ValueError('바퀴 반지름·방향 설정 오류')
        problem = port_problem(settings['motor_port'])
        if problem:
            raise RuntimeError(problem)
        bus = MotorBus(settings)
        for motor_id in ids:
            bus.ping(motor_id)
            registers = {name: bus.read(motor_id, address, size) for name, address, size in (
                ('firmware', 6, 1), ('mode', 11, 1), ('torque', 64, 1),
                ('return_level', 68, 1), ('watchdog', 98, 1), ('profile_acceleration', 108, 4))}
            sample = bus.snapshot(motor_id)
            recorder.event('motor_initial', **sample, registers=registers)
            originals[motor_id] = registers
            if (registers['torque'] != 0 or registers['firmware'] < 38
                    or registers['return_level'] != 2 or registers['watchdog'] == 255
                    or sample['hardware_error'] or sample['velocity_raw'] != 0):
                raise RuntimeError(f'ID {motor_id}: 정지·토크·통신·오류 사전 검사 실패')
        for motor_id in ids:
            changed.append(motor_id)
            if originals[motor_id]['mode'] != 1:
                bus.write(motor_id, 11, 1)
            bus.write(motor_id, 104, 0, 4)
            bus.write(motor_id, 108, 5, 4)
            bus.write(motor_id, 98, 15)
        for motor_id in ids:
            bus.write(motor_id, 64, 1)
        baseline = [bus.snapshot(i)['position_raw'] for i in ids]
        expected_seconds = args.distance / (args.velocity_raw * VELOCITY_UNIT * radius)
        deadline = time.monotonic() + min(300, expected_seconds * 1.6 + 20)
        stall_time = [time.monotonic(), time.monotonic()]
        stall_reference = [0.0, 0.0]
        next_print = 0
        print(f'전진 시작: 목표 {args.distance:.3f}m, 속도 상한 {args.velocity_raw}, '
              f'예상 약 {expected_seconds:.0f}초', flush=True)
        while True:
            tick = time.monotonic()
            samples = [bus.snapshot(i) for i in ids]
            distances = wheel_distances(baseline, [s['position_raw'] for s in samples], radius, directions)
            progress = sum(distances) / 2
            recorder.event('distance_sample', left_m=distances[0], right_m=distances[1],
                           average_m=progress, motors=samples)
            if any(s['hardware_error'] for s in samples):
                raise RuntimeError('주행 중 모터 하드웨어 오류')
            if min(distances) < -0.005:
                raise RuntimeError('역방향 엔코더 움직임 감지')
            if abs(distances[0] - distances[1]) > 0.03:
                raise RuntimeError('좌우 주행 거리 차이가 3cm를 초과하여 정지')
            if progress >= args.distance:
                success, reason = True, '엔코더 기준 목표 거리 도달'
                break
            if tick > deadline:
                raise RuntimeError('주행 시간 제한 초과')
            for side in (0, 1):
                if distances[side] - stall_reference[side] >= 0.001:
                    stall_reference[side], stall_time[side] = distances[side], tick
                elif tick - stall_time[side] > 3:
                    raise RuntimeError(f'ID {ids[side]}: 3초 동안 유효한 전진 없음')
            raw_goals = goals_for_distance(distances, args.distance - progress, args.velocity_raw)
            for motor_id, raw, direction in zip(ids, raw_goals, directions):
                bus.write(motor_id, 104, raw * direction, 4)
            if tick >= next_print:
                print(f'전진 거리: 평균 {progress:.3f}m '
                      f'(왼쪽 {distances[0]:.3f} / 오른쪽 {distances[1]:.3f})', flush=True)
                next_print = tick + 5
            time.sleep(max(0, 0.1 - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        success, reason = False, '사용자 중단'
    except Exception as exc:
        success, reason = False, str(exc)
    finally:
        if bus:
            if changed:
                # Decelerate at zero target while keeping the watchdog alive.
                errors = []
                try:
                    for motor_id in ids:
                        bus.write(motor_id, 104, 0, 4)
                    settle_until = time.monotonic() + 0.5
                    while time.monotonic() < settle_until:
                        samples = [bus.snapshot(i) for i in ids]
                        time.sleep(0.05)
                    if baseline is not None:
                        distances = wheel_distances(
                            baseline, [s['position_raw'] for s in samples], radius, directions)
                    if any(abs(s['velocity_raw']) > 2 for s in samples):
                        errors.append('목표 속도 0 후 회전 지속')
                except Exception as exc:
                    errors.append(str(exc))
                errors.extend(stop_motors(bus, changed, recorder))
                if not errors:
                    for motor_id in changed:
                        try:
                            original = originals[motor_id]
                            if original['mode'] != 1:
                                bus.write(motor_id, 11, original['mode'])
                            bus.write(motor_id, 104, 0, 4)
                            bus.write(motor_id, 108, original['profile_acceleration'], 4)
                            bus.write(motor_id, 98, original['watchdog'])
                            if bus.read(motor_id, 64) != 0 or bus.read(motor_id, 104, 4) != 0:
                                raise RuntimeError(f'ID {motor_id}: 복구 후 토크/목표 속도 확인 실패')
                        except Exception as exc:
                            errors.append(str(exc))
                if errors:
                    success, reason = False, '정지/설정 복구 확인 실패: ' + '; '.join(errors)
            bus.close()
    recorder.result('drive_distance', 'pass' if success else 'fail', reason=reason,
                    left_m=distances[0], right_m=distances[1], average_m=sum(distances)/2,
                    measured_by='wheel encoders; physical displacement not independently measured')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=1.0)
    parser.add_argument('--velocity-raw', type=int, default=15)
    parser.add_argument('--placement', required=True, choices=('floor-clear', 'wheels-lifted'))
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    if not 0 < args.distance <= 1.0 or not 1 <= args.velocity_raw <= 15:
        parser.error('거리는 0~1m, 속도 원시값은 1~15 범위여야 합니다')
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = Path(get_package_share_directory('orinbot_hardware')) / 'config/hardware.yaml'
    settings = yaml.safe_load(args.config.read_text())
    directory = Path.home() / 'ros2_ws/log/distance_tests' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    recorder = Recorder(directory)
    recorder.report['settings'] = settings
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    print(f'로그 폴더: {directory}', flush=True)
    drive(settings, args, recorder)
    return recorder.finish()


if __name__ == '__main__':
    sys.exit(main())
