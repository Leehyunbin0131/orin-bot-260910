import importlib.util
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('interfaces_runner', ROOT / 'scripts/test_interfaces.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeBus:
    def __init__(self, fail_enable=None):
        self.registers = {i: {6: 45, 10: 0, 11: 3, 64: 0, 68: 2, 98: 0,
                              104: 0, 108: 7} for i in (1, 2)}
        self.positions = {1: 0, 2: 0}
        self.writes = []
        self.closed = False
        self.fail_enable = fail_enable

    def ping(self, motor_id):
        return 1030

    def read(self, motor_id, address, size=1):
        return self.registers[motor_id][address]

    def write(self, motor_id, address, value, size=1):
        self.writes.append((motor_id, address, value))
        if motor_id == self.fail_enable and address == 64 and value == 1:
            raise RuntimeError('Injected enable failure')
        self.registers[motor_id][address] = value
        if address == 11:
            # Real XM430 resets Goal Velocity when Operating Mode is changed.
            self.registers[motor_id][104] = 330

    def snapshot(self, motor_id):
        velocity = self.registers[motor_id][104]
        self.positions[motor_id] += velocity
        return {'id': motor_id, 'velocity_raw': velocity,
                'position_raw': self.positions[motor_id], 'hardware_error': 0}

    def close(self):
        self.closed = True


class InterfaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.recorder = runner.Recorder(Path(self.temp.name))
        self.settings = yaml.safe_load((ROOT / 'config/hardware.yaml').read_text())

    def tearDown(self):
        self.recorder.events.close()
        self.temp.cleanup()

    def run_motor(self, bus, motion=False):
        args = SimpleNamespace(motion=motion, velocity_raw=15, motion_seconds=0.01)
        with patch.object(runner, 'port_problem', return_value=None), \
                patch.object(runner.importlib.util, 'find_spec', return_value=True), \
                patch.object(runner, 'MotorBus', return_value=bus):
            runner.motor_test(self.settings, args, self.recorder)

    def test_default_motor_check_has_no_writes(self):
        bus = FakeBus()
        self.run_motor(bus)
        self.assertEqual(bus.writes, [])
        self.assertTrue(bus.closed)
        self.assertEqual(self.recorder.report['interfaces']['motors']['status'], 'pass')

    def test_cli_rejects_speed_above_verified_limit(self):
        with patch('sys.argv', ['test_interfaces.py', '--velocity-raw', '16']), \
                redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
            runner.parse_args()
        self.assertEqual(raised.exception.code, 2)

    def test_rotation_checks_encoder_then_stops_both_and_restores_mode(self):
        bus = FakeBus()
        self.run_motor(bus, motion=True)
        result = self.recorder.report['interfaces']['motors']
        self.assertEqual(result['status'], 'pass')
        self.assertEqual(len(result['phases']), 4)
        for motor_id in (1, 2):
            self.assertEqual(bus.registers[motor_id][64], 0)
            self.assertEqual(bus.registers[motor_id][104], 0)
            self.assertEqual(bus.registers[motor_id][11], 3)
            self.assertEqual(bus.registers[motor_id][108], 7)
        self.assertTrue(bus.closed)

    def test_partial_enable_failure_stops_both_motors(self):
        bus = FakeBus(fail_enable=2)
        self.run_motor(bus, motion=True)
        self.assertEqual(self.recorder.report['interfaces']['motors']['status'], 'fail')
        for motor_id in (1, 2):
            self.assertIn((motor_id, 64, 0), bus.writes)
            self.assertEqual(bus.registers[motor_id][64], 0)
        self.assertTrue(bus.closed)

    def test_failed_stop_of_one_motor_does_not_skip_other_motor(self):
        bus = FakeBus()
        original_write = bus.write
        def failed_write(motor_id, address, value, size=1):
            if motor_id == 1:
                raise RuntimeError('Motor 1 disconnected')
            original_write(motor_id, address, value, size)
        bus.write = failed_write
        errors = runner.stop_motors(bus, (1, 2), self.recorder)
        self.assertTrue(errors)
        self.assertIn((2, 64, 0), bus.writes)

    def test_second_motor_preflight_failure_prevents_all_writes(self):
        bus = FakeBus()
        bus.registers[2][64] = 1
        self.run_motor(bus, motion=True)
        self.assertEqual(bus.writes, [])
        self.assertEqual(self.recorder.report['interfaces']['motors']['status'], 'fail')

    def test_scan_all_invalid_is_not_a_pass(self):
        sample = SimpleNamespace(ranges=[float('inf'), float('nan'), 0.0], range_min=0.1, range_max=12)
        self.assertFalse(runner.sensor_summary('scan', sample)['valid'])

    def test_zero_depth_and_invalid_imu_are_not_a_pass(self):
        sample = SimpleNamespace(width=16, height=16, step=32, encoding='16UC1',
                                 is_bigendian=False, data=bytes(512))
        self.assertFalse(runner.sensor_summary('depth', sample)['valid'])
        sample = SimpleNamespace(angular_velocity=SimpleNamespace(x=float('nan'), y=0, z=0),
                                 linear_acceleration=SimpleNamespace(x=0, y=0, z=9.81))
        self.assertFalse(runner.sensor_summary('imu', sample)['valid'])

    def test_real_ros_subscriptions_collect_synthetic_sensor_messages(self):
        import rclpy
        from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        create_node = rclpy.create_node

        def create_publishing_node(*args, **kwargs):
            node = create_node(*args, **kwargs)
            color = Image(width=16, height=16, step=48, encoding='rgb8', data=bytes(768))
            depth = Image(width=16, height=16, step=32, encoding='16UC1',
                          data=bytes([232, 3]) * 256)
            imu = Imu()
            imu.linear_acceleration.z = 9.81
            scan = LaserScan(range_min=0.1, range_max=12.0, ranges=[1.0, 2.0])
            points = PointCloud2(width=1, height=1, data=bytes(16))
            streams = [('/scan', scan), ('/camera/color/image_raw', color),
                       ('/camera/aligned_depth_to_color/image_raw', depth),
                       ('/camera/imu', imu), ('/camera/depth/color/points', points)]
            publishers = [(node.create_publisher(type(m), topic, qos_profile_sensor_data), m)
                          for topic, m in streams]
            def publish():
                for publisher, message in publishers:
                    message.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(message)
            node.create_timer(0.05, publish)
            return node

        args = SimpleNamespace(domain=177, duration=3.2, config=ROOT / 'config/hardware.yaml')
        fake_process = SimpleNamespace(pid=99999999, poll=lambda: None, wait=lambda **kw: 0)
        with patch.object(rclpy, 'create_node', side_effect=create_publishing_node), \
                patch('ament_index_python.packages.get_package_prefix', return_value='/opt/ros/jazzy'), \
                patch.object(runner, 'port_problem', return_value=None), \
                patch.object(runner.subprocess, 'Popen', return_value=fake_process), \
                patch.object(runner.os, 'killpg'):
            runner.sensor_test(self.settings, args, self.recorder)
        for name in ('camera', 'lidar'):
            self.assertEqual(self.recorder.report['interfaces'][name]['status'], 'pass')
        self.assertGreater(self.recorder.report['interfaces']['lidar']['topics']['/scan']['hz'], 10)


if __name__ == '__main__':
    unittest.main()
