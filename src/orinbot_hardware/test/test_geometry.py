"""Measured real geometry and the axle-centered odometry frame; no hardware I/O."""
import importlib.util
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('real_base_geometry', ROOT / 'launch/base.launch.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


class RealGeometryTests(unittest.TestCase):
    def setUp(self):
        self.settings = yaml.safe_load((ROOT / 'config/hardware.yaml').read_text())
        self.robot = base.build_geometry(self.settings)

    def position(self, name):
        # All ancestors up to these physical reference points have zero rotation.
        if name == 'base_footprint':
            return (0., 0., 0.)
        joint = next(j for j in self.robot.findall('joint') if j.find('child').get('link') == name)
        parent = self.position(joint.find('parent').get('link'))
        return tuple(a+b for a, b in zip(parent, map(float, joint.find('origin').get('xyz').split())))

    def test_body_bottom_and_center_match_measured_clearance(self):
        center = self.position('base_link')
        self.assertAlmostEqual(center[0], -0.14)
        self.assertAlmostEqual(center[1], 0.)
        self.assertAlmostEqual(center[2], 0.278)
        bars = self.robot.findall("link[@name='base_link']/collision")
        self.assertEqual(len(bars), 12)
        bounds = [[], [], []]
        for bar in bars:
            xyz = list(map(float, bar.find('origin').get('xyz').split()))
            size = list(map(float, bar.find('geometry/box').get('size').split()))
            for axis in range(3):
                bounds[axis].extend((xyz[axis]-size[axis]/2, xyz[axis]+size[axis]/2))
        for limits, expected in zip(bounds, (0.44, 0.44, self.settings['body_height'])):
            self.assertAlmostEqual(max(limits)-min(limits), expected)
        self.assertAlmostEqual(self.settings['body_length']-2*self.settings['profile_size'], 0.40)
        self.assertAlmostEqual(self.settings['body_width']-2*self.settings['profile_size'], 0.40)
        self.assertAlmostEqual(self.settings['body_height']-2*self.settings['profile_size'], 0.40)
        self.assertAlmostEqual(center[2]-0.44/2, 0.058)

    def test_drive_axle_is_odom_origin_and_track_is_44cm(self):
        left, right = self.position('left_wheel_link'), self.position('right_wheel_link')
        self.assertAlmostEqual(left[0], 0.)
        self.assertAlmostEqual(right[0], 0.)
        self.assertAlmostEqual(left[1], 0.22)
        self.assertAlmostEqual(right[1], -0.22)
        self.assertAlmostEqual(left[2], self.settings['wheel_radius'])
        self.assertAlmostEqual(left[1]-right[1], self.settings['wheel_separation'])

    def test_two_rear_caster_axes_follow_rear_profile_centerline(self):
        names = [link.get('name') for link in self.robot.findall('link') if link.get('name').endswith('_caster_ground')]
        self.assertEqual(set(names), {'rear_left_caster_ground', 'rear_right_caster_ground'})
        for side, y in (('left', 0.13), ('right', -0.13)):
            xyz = self.position(f'rear_{side}_caster_ground')
            for actual, expected in zip(xyz, (-0.35, y, 0.)):
                self.assertAlmostEqual(actual, expected)

    def test_changing_axle_position_preserves_body_relative_sensor_position(self):
        self.settings['drive_axle_x'] = 0.10
        self.robot = base.build_geometry(self.settings)
        for name in ('laser', 'camera_link'):
            sensor, body = self.position(name), self.position('base_link')
            self.assertAlmostEqual(sensor[0], 0.11)
            self.assertAlmostEqual(sensor[0]-body[0], 0.21)

    def test_motor_mount_reference_is_on_side_profile_center(self):
        for side, y in (('left', 0.21), ('right', -0.21)):
            xyz = self.position(f'{side}_motor_mount')
            self.assertAlmostEqual(xyz[0], 0.)
            self.assertAlmostEqual(xyz[1], y)
            self.assertAlmostEqual(xyz[2], 0.068)

    def test_preview_has_no_hardware_plugins_or_unmeasured_geometry(self):
        self.assertEqual(self.robot.findall('ros2_control'), [])
        self.assertEqual(self.robot.findall('gazebo'), [])
        self.assertEqual(self.robot.findall('.//inertial'), [])
        for side in ('left', 'right'):
            if self.settings['wheel_width'] is None:
                self.assertIsNone(self.robot.find(f"link[@name='{side}_wheel_link']/collision"))

    def test_caster_dimensions_and_mounting_gap_match_supplied_spec(self):
        for side in ('left', 'right'):
            mount = self.position(f'rear_{side}_caster_mount')
            wheel = self.position(f'rear_{side}_caster_wheel_link')
            self.assertAlmostEqual(mount[2], 0.042)
            self.assertAlmostEqual(0.058-mount[2], 0.016)
            self.assertAlmostEqual(wheel[2], 0.0162)
            cylinder = self.robot.find(f"link[@name='rear_{side}_caster_wheel_link']/visual/geometry/cylinder")
            self.assertAlmostEqual(float(cylinder.get('radius'))*2, 0.0324)
            self.assertAlmostEqual(float(cylinder.get('length')), 0.0142)

    def test_measured_wheel_width_enables_matching_visual_and_collision(self):
        self.settings['wheel_width'] = 0.025
        self.robot = base.build_geometry(self.settings)
        for kind in ('visual', 'collision'):
            cylinder = self.robot.find(f"link[@name='left_wheel_link']/{kind}/geometry/cylinder")
            self.assertAlmostEqual(float(cylinder.get('length')), 0.025)

    def test_geometry_tree_is_connected_and_has_one_parent_per_link(self):
        links = {link.get('name') for link in self.robot.findall('link')}
        joints = self.robot.findall('joint')
        children = [j.find('child').get('link') for j in joints]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links-set(children), {'base_footprint'})
        reached = {'base_footprint'}
        for _ in links:
            reached |= {j.find('child').get('link') for j in joints if j.find('parent').get('link') in reached}
        self.assertEqual(reached, links)

    def test_invalid_geometry_is_rejected(self):
        for change in ({'ground_clearance': -0.1}, {'body_width': float('nan')},
                       {'caster_x': 0.30}, {'drive_axle_x': float('inf')},
                       {'wheel_width': -0.1}, {'caster_radius': 0}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                base.build_geometry({**self.settings, **change})


if __name__ == '__main__':
    unittest.main()
