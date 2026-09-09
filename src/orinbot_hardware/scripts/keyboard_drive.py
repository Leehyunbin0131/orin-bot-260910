#!/usr/bin/env python3
"""Terminal deadman control: fixed 50rpm, translation OR in-place rotation."""
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

WHEEL_RAD_S = 218 * 0.0239691227  # XM430 0.229rpm units: 49.922rpm
KEY_TIMEOUT = 0.25


def key_velocity(key, radius, separation):
    if not all(math.isfinite(v) and v > 0 for v in (radius, separation)):
        raise ValueError('Wheel radius and separation must be positive and finite')
    linear = WHEEL_RAD_S * radius
    angular = 2 * linear / separation
    return {'i': (linear, 0.0), ',': (-linear, 0.0),
            'j': (0.0, angular), 'l': (0.0, -angular)}.get(key, (0.0, 0.0))


class KeyboardState:
    def __init__(self, radius, separation):
        key_velocity('', radius, separation)
        self.radius, self.separation = radius, separation
        self.last_key = -math.inf
        self.velocity = (0.0, 0.0)

    def press(self, key, now):
        self.velocity = key_velocity(key, self.radius, self.separation)
        self.last_key = now

    def command(self, now):
        return self.velocity if 0 <= now - self.last_key < KEY_TIMEOUT else (0.0, 0.0)


def main():
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions

    if not sys.stdin.isatty():
        raise SystemExit('Interactive terminal required (use ssh -t for remote commands).')
    # Let Python handle Ctrl+C so ROS stays alive for the final stop messages.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('keyboard_drive')
    default = str(Path(get_package_share_directory('orinbot_hardware')) / 'config/hardware.yaml')
    config = node.declare_parameter('config', default).value
    with open(config) as source:
        settings = yaml.safe_load(source)
    state = KeyboardState(float(settings['wheel_radius']), float(settings['wheel_separation']))
    publisher = node.create_publisher(TwistStamped, '/keyboard/cmd_vel', 1)

    def publish(velocity):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x, message.twist.angular.z = velocity
        publisher.publish(message)

    print('50rpm 고정: i 전진 / , 후진 / j 좌회전 / l 우회전 / k 또는 Space 정지 / Ctrl+C 종료')
    print('방향 키를 누르고 있으세요. 0.25초 동안 반복 입력이 없으면 정지합니다.')
    print('베이스는 keyboard_mode:=true로 실행해야 합니다. 맵 작성 런치는 이를 포함합니다.')
    print('motion_service.launch.py 실행 중에는 키보드 입력이 연결되지 않습니다.', flush=True)
    next_connection_check = 0.0
    connected = None
    last_velocity = None
    original = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if now >= next_connection_check:
                receivers = node.get_subscriptions_info_by_topic(publisher.topic_name)
                found = any(info.node_name == 'diff_drive_controller' for info in receivers)
                if found and connected is not True:
                    print('연결 확인: 키보드 토픽을 diff_drive_controller가 구독합니다.', flush=True)
                elif not found:
                    print('연결 대기: 키보드 토픽을 받는 제어기가 없습니다. '
                          '서비스 주행 런치를 종료하고 베이스를 keyboard_mode:=true로 실행하세요. '
                          '두 터미널 모두 jazzy 환경(ROS_DOMAIN_ID=11)이 필요합니다.', flush=True)
                connected = found
                next_connection_check = now + 5.0
            readable, _, _ = select.select([sys.stdin], [], [], 1 / 30)
            if readable:
                # Read from the descriptor used by select, without TextIO buffering.
                key = os.read(sys.stdin.fileno(), 1).decode('ascii', errors='replace')
                if not key or key == '\x03':
                    break
                state.press(key, time.monotonic())
            velocity = state.command(time.monotonic())
            if velocity != last_velocity:
                label = {(0.0, 0.0): '정지'}
                for key, name in (('i', '전진'), (',', '후진'), ('j', '좌회전'), ('l', '우회전')):
                    label[key_velocity(key, state.radius, state.separation)] = name
                print(f'키보드 명령: {label[velocity]} (입력 확인; 실제 이동 확인은 아님)', flush=True)
                last_velocity = velocity
            publish(velocity)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        # If ROS has already shut down, the controller's 0.3s watchdog stops it.
        if rclpy.ok():
            for _ in range(12):
                publish((0.0, 0.0))
                time.sleep(1 / 30)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
