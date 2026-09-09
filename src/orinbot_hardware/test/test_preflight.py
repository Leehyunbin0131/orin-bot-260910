import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('base_launch', ROOT / 'launch/base.launch.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


class ReadOnlyBus:
    def __init__(self, register_changes=None, model=1030, velocity=0, communication=0,
                 baud=57600, packet_error=0):
        self.registers = {6: 45, 64: 0, 68: 2, 70: 0, 98: 0}
        self.registers.update(register_changes or {})
        self.model, self.velocity, self.communication = model, velocity, communication
        self.packet_error = packet_error
        self.closed = False
        self.baud = baud
        self.requested_baud = None
        self.pinged_ids = []

    def openPort(self): return True
    def setBaudRate(self, baud):
        self.requested_baud = baud
        return baud == self.baud
    def closePort(self): self.closed = True
    def ping(self, port, motor_id):
        self.pinged_ids.append(motor_id)
        return self.model, self.communication, self.packet_error
    def read1ByteTxRx(self, port, motor_id, address): return self.registers[address], 0, self.packet_error
    def read4ByteTxRx(self, port, motor_id, address): return self.velocity, 0, 0
    # No write methods: the preflight must remain read-only.


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.settings = yaml.safe_load((ROOT / 'config/hardware.yaml').read_text())

    def check_bus(self, bus):
        sdk = SimpleNamespace(COMM_SUCCESS=0, PortHandler=lambda _: bus, PacketHandler=lambda _: bus)
        with patch.dict('sys.modules', {'dynamixel_sdk': sdk}), patch.object(base.os, 'access', return_value=True):
            base.verify_bus(self.settings)

    def test_stationary_motors_can_be_checked_without_writes(self):
        bus = ReadOnlyBus()
        self.check_bus(bus)
        self.assertTrue(bus.closed)
        self.assertEqual(bus.requested_baud, 57600)
        self.assertEqual(bus.pinged_ids, [1, 2])

    def test_selected_baud_is_forwarded_to_bus(self):
        self.settings['baud_rate'] = 115200
        bus = ReadOnlyBus(baud=115200)
        self.check_bus(bus)
        self.assertEqual(bus.requested_baud, 115200)

    def test_failed_port_setup_is_closed(self):
        bus = ReadOnlyBus(baud=9600)
        with self.assertRaisesRegex(RuntimeError, 'Could not open motor port'):
            self.check_bus(bus)
        self.assertTrue(bus.closed)

    def test_unsafe_or_incompatible_bus_is_rejected_and_closed(self):
        for bus, reason in ((ReadOnlyBus({64: 1}), 'torque=1'),
                            (ReadOnlyBus({70: 4}), 'hardware_error=4'),
                            (ReadOnlyBus({98: 255}), 'watchdog=255'),
                            (ReadOnlyBus({6: 37}), 'firmware=37'),
                            (ReadOnlyBus({6: 44}), 'firmware=44'),
                            (ReadOnlyBus({68: 1}), 'return_level=1'),
                            (ReadOnlyBus(model=1020), 'XM430-W210 not confirmed'),
                            (ReadOnlyBus(velocity=1), 'wheel is moving'),
                            (ReadOnlyBus(communication=-3001), 'communication failed'),
                            (ReadOnlyBus({70: 32}, packet_error=0x80), 'hardware_error=32'),
                            (ReadOnlyBus(packet_error=0x07), 'packet_error=0x07')):
            with self.subTest(bus=vars(bus)):
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.check_bus(bus)
                self.assertTrue(bus.closed)

    def test_real_description_keeps_torque_off_and_uses_requested_motor_settings(self):
        robot = ET.fromstring(base.build_description(self.settings))
        hardware = robot.find('ros2_control/hardware')
        self.assertEqual(hardware.findtext('plugin'),
                         'dynamixel_hardware_interface/DynamixelHardware')
        self.assertEqual(hardware.findtext("param[@name='baud_rate']"), '57600')
        for gpio, expected_id in zip(robot.findall('ros2_control/gpio'), (1, 2)):
            self.assertEqual(gpio.findtext("param[@name='ID']"), str(expected_id))
            self.assertEqual(gpio.findtext("param[@name='Torque Enable']"), '0')
            self.assertEqual(gpio.findtext("param[@name='Goal Velocity']"), '0')
            self.assertEqual(gpio.findtext("param[@name='Bus Watchdog']"), '15')
            interfaces = {item.get('name') for item in gpio.findall('state_interface')}
            self.assertTrue({'Torque Enable', 'Goal Velocity'}.issubset(interfaces))
        self.assertEqual(len(robot.findall('ros2_control/gpio')), 2)

    def test_mock_description_contains_only_mock_hardware(self):
        robot = ET.fromstring(base.build_description(self.settings, mock=True))
        self.assertEqual(robot.findtext('ros2_control/hardware/plugin'),
                         'mock_components/GenericSystem')
        self.assertEqual(robot.findall('ros2_control/gpio'), [])
        self.assertEqual(robot.findtext("ros2_control/hardware/param[@name='calculate_dynamics']"), 'true')

    def test_sensor_mounts_use_floor_heights_and_leave_internal_tf_to_driver(self):
        robot = ET.fromstring(base.build_description(self.settings, mock=True))
        for name, height in (('lidar', 0.52), ('camera', 0.45)):
            mount = robot.find(f"joint[@name='{name}_mount_joint']")
            self.assertEqual(mount.find('parent').get('link'), 'base_footprint')
            xyz = [float(v) for v in mount.find('origin').get('xyz').split()]
            self.assertAlmostEqual(xyz[0], 0.07)
            self.assertEqual(xyz[1:], [0.0, height])
        pitch = float(robot.find("joint[@name='camera_mount_joint']/origin").get('rpy').split()[1])
        self.assertAlmostEqual(pitch, math.radians(15))
        self.assertLess(-math.sin(pitch), 0, 'Camera forward axis must point downward')
        camera_links = [link.get('name') for link in robot.findall('link')
                        if link.get('name').startswith('camera')]
        self.assertEqual(camera_links, ['camera_link'])
        self.assertEqual(robot.findall('gazebo'), [])

    def test_rear_facing_laser_axis_is_transformed_to_robot_forward(self):
        robot = ET.fromstring(base.build_description(self.settings, mock=True))
        yaw = float(robot.find("joint[@name='lidar_mount_joint']/origin").get('rpy').split()[2])
        # Forward robot displacement appears along -x in this mounted laser frame.
        self.assertAlmostEqual(-0.20*math.cos(yaw), 0.20)
        self.assertAlmostEqual(-0.20*math.sin(yaw), 0.0)

    def test_bad_wheel_configuration_is_rejected(self):
        for changes in ({'right_id': 1}, {'wheel_radius': float('nan')}, {'wheel_separation': 0},
                        {'left_direction': 0}, {'left_id': 253}, {'baud_rate': 12345},
                        {'camera_z': -0.1}, {'lidar_x': float('nan')},
                        {'camera_pitch_degrees': float('inf')}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    base.validate_settings({**self.settings, **changes})

    def test_combined_controller_limits_respect_verified_wheel_speed(self):
        parameters = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())[
            'diff_drive_controller']['ros__parameters']
        base.validate_controller_limits(self.settings, parameters)
        # A turn can over-speed one wheel even if straight driving alone is safe.
        for change in ({'angular.z.max_velocity': 0.1},
                       {'linear.x.min_velocity': -0.05},
                       {'linear.x.has_velocity_limits': False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                base.validate_controller_limits(self.settings, {**parameters, **change})

    def test_explicit_60rpm_test_is_straight_only_and_bounded(self):
        parameters = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())[
            'diff_drive_controller']['ros__parameters']
        self.assertEqual(base.configure_straight_speed_test(self.settings, parameters, 60), 262)
        self.assertEqual(parameters['angular.z.max_velocity'], 0.0)
        self.assertAlmostEqual(parameters['linear.x.max_velocity'] / self.settings['wheel_radius'],
                               262 * 0.0239691227)
        for rpm in (61, -1, float('nan'), float('inf')):
            with self.subTest(rpm=rpm), self.assertRaises(ValueError):
                base.configure_straight_speed_test(self.settings, parameters, rpm)

    def test_rectangle_profile_has_50rpm_straights_and_slow_corners(self):
        parameters = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())[
            'diff_drive_controller']['ros__parameters']
        base.configure_rectangle_profile(self.settings, parameters, 50)
        self.assertAlmostEqual(parameters['linear.x.max_velocity'], 218*0.0239691227*0.0325)
        self.assertLess(parameters['angular.z.max_velocity'], 0.16)
        base.validate_controller_limits(self.settings, parameters, 262)
        with self.assertRaises(ValueError):
            base.configure_rectangle_profile(self.settings, parameters, 61)

    def test_fused_profile_accepts_60rpm_and_25rpm_turns(self):
        parameters = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())[
            'diff_drive_controller']['ros__parameters']
        base.configure_rectangle_profile(self.settings, parameters, 60, 25)
        self.assertAlmostEqual(parameters['linear.x.max_velocity'], 262*0.0239691227*0.0325)
        self.assertAlmostEqual(parameters['angular.z.max_velocity'],
                               2*109*0.0239691227*0.0325/self.settings['wheel_separation'])
        self.assertEqual(parameters['angular.z.max_acceleration'], 0.25)
        self.assertEqual(parameters['angular.z.max_deceleration'], -0.25)
        base.configure_rectangle_profile(self.settings, parameters, 5, 25)
        self.assertAlmostEqual(parameters['angular.z.max_velocity'],
                               2*21*0.0239691227*0.0325/self.settings['wheel_separation'])
        for rpm, turn in ((61, 25), (60, 26), (60, 0), (60, float('nan'))):
            with self.subTest(rpm=rpm, turn=turn), self.assertRaises(ValueError):
                base.configure_rectangle_profile(self.settings, parameters, rpm, turn)

    def test_keyboard_profile_has_50rpm_on_both_axes_and_matched_slew(self):
        parameters = yaml.safe_load((ROOT / 'config/controllers.yaml').read_text())[
            'diff_drive_controller']['ros__parameters']
        base.configure_keyboard_profile(self.settings, parameters)
        linear = parameters['linear.x.max_velocity']
        angular = parameters['angular.z.max_velocity']
        self.assertAlmostEqual(linear / .0325, 218 * .0239691227)
        self.assertAlmostEqual(angular * .22 / .0325, 218 * .0239691227)
        for axis in ('linear.x', 'angular.z'):
            vmax = parameters[f'{axis}.max_velocity']
            self.assertAlmostEqual(vmax / parameters[f'{axis}.max_acceleration'], 4)
            self.assertAlmostEqual(vmax / -parameters[f'{axis}.max_deceleration'], 1)


if __name__ == '__main__':
    unittest.main()
