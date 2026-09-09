import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


keyboard = module('keyboard_drive')
front = module('front_scan')


class KeyboardTests(unittest.TestCase):
    def test_each_motion_key_commands_both_wheels_at_50rpm(self):
        for key, signs in (('i', (1, 1)), (',', (-1, -1)), ('j', (-1, 1)), ('l', (1, -1))):
            v, w = keyboard.key_velocity(key, .0325, .44)
            for actual, sign in zip(((v - w * .22) / .0325, (v + w * .22) / .0325), signs):
                # Installed driver uses an approximate rad/s conversion. Its raw
                # 218 is 49.922rpm nominal, ~49.898rpm in ROS joint feedback.
                self.assertAlmostEqual(actual * 60 / (2 * math.pi), sign * 49.922, delta=.03)
                self.assertAlmostEqual(actual / .0239691227, sign * 218)

    def test_speed_change_diagonal_and_unknown_keys_stop(self):
        for key in 'qzwecxuom. k\x1b':
            self.assertEqual(keyboard.key_velocity(key, .0325, .44), (0, 0))

    def test_startup_release_repeat_and_stop(self):
        state = keyboard.KeyboardState(.0325, .44)
        self.assertEqual(state.command(0), (0, 0))
        state.press('i', 10)
        self.assertGreater(state.command(10.2)[0], 0)
        self.assertEqual(state.command(10.251), (0, 0))
        state.press('j', 11)
        self.assertGreater(state.command(11.1)[1], 0)
        state.press('k', 11.11)
        self.assertEqual(state.command(11.12), (0, 0))


class FrontScanTests(unittest.TestCase):
    def scan(self, start=-180, step=1, count=361):
        return SimpleNamespace(header=SimpleNamespace(frame_id='laser', stamp=123),
                               angle_min=math.radians(start), angle_increment=math.radians(step),
                               time_increment=.001, ranges=[2.] * count, intensities=[7.] * count)

    def test_rear_facing_mount_keeps_robot_front_and_sides(self):
        scan = front.mask_scan(self.scan(), 180, 270)
        # laser ±180 = body front; laser 0 = body rear.
        for index in (0, 90, 135, 225, 270, 360):
            self.assertEqual(scan.ranges[index], 2)
        for index in range(136, 225):
            self.assertTrue(math.isnan(scan.ranges[index]))
            self.assertEqual(scan.intensities[index], 0)
        self.assertEqual(len(scan.ranges), 361)
        self.assertEqual(scan.header.stamp, 123)
        self.assertEqual(scan.time_increment, .001)

    def test_forward_mount_masks_ends_without_reordering(self):
        scan = front.mask_scan(self.scan(), 0, 270)
        self.assertEqual(scan.ranges[180], 2)
        self.assertTrue(math.isnan(scan.ranges[0]))
        self.assertTrue(math.isnan(scan.ranges[360]))

    def test_clockwise_scan_empty_intensities_and_existing_invalid_ranges(self):
        scan = self.scan(180, -1)
        scan.intensities = []
        scan.ranges[0] = float('inf')
        scan.ranges[1] = float('nan')
        front.mask_scan(scan, 180, 270)
        self.assertTrue(math.isinf(scan.ranges[0]))
        self.assertTrue(math.isnan(scan.ranges[1]))
        self.assertTrue(math.isnan(scan.ranges[180]))
        self.assertEqual(scan.ranges[90], 2)

    def test_invalid_configuration_and_wrong_frame_rejected(self):
        for yaw, fov in ((math.nan, 270), (0, 0), (0, 361), (0, math.inf)):
            with self.assertRaises(ValueError):
                front.mask_scan(self.scan(), yaw, fov)
        scan = self.scan()
        scan.header.frame_id = 'base_footprint'
        with self.assertRaises(ValueError):
            front.mask_scan(scan, 180, 270)


if __name__ == '__main__':
    unittest.main()
