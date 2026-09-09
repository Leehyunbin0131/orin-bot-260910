"""Sensor loss must stop motion even while the EKF keeps producing predictions."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rtabmap_msgs.msg import OdomInfo

scripts = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(scripts))
import drive_fused_rectangle as fused


def message_for(kind, stamp=100.0):
    message = fused.SOURCES[kind][0]()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    if isinstance(message, Odometry):
        message.child_frame_id = 'base_footprint'
        message.pose.pose.orientation.w = 1.0
        for i in (0, 7, 35):
            message.pose.covariance[i] = 0.01
    elif isinstance(message, Imu):
        message.header.frame_id = 'camera_imu_optical_frame'
        message.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            message.angular_velocity_covariance[i] = 0.01
    elif isinstance(message, LaserScan):
        message.header.frame_id = 'laser'
        message.range_min, message.range_max = 0.1, 10.0
        message.ranges = [2.0]*360
    else:
        message.icp_inliers_ratio = 0.9
    return message


def ready_health():
    health = fused.InputHealth()
    for i in range(5):
        stamp = 100+i*0.1
        for kind in fused.SOURCES:
            health.update(kind, message_for(kind, stamp), stamp, stamp)
    health.armed = True
    return health


class FusionGuardTests(unittest.TestCase):
    def test_any_missing_sensor_blocks_startup(self):
        for missing in fused.SOURCES:
            health = ready_health()
            health.states[missing]['valid'] = False
            self.assertIn(missing, health.problem(100.4, 100.4))

    def test_live_ekf_predictions_do_not_hide_input_loss(self):
        for stopped in ('wheel', 'imu', 'scan', 'lidar', 'tracking'):
            health = ready_health()
            for kind in fused.SOURCES:
                if kind != stopped:
                    health.update(kind, message_for(kind, 102.0), 102.0, 102.0)
            self.assertIn(stopped, health.problem(102.0, 102.0))

    def test_failed_lidar_tracking_latches_until_test_ends(self):
        health = ready_health()
        message = message_for('tracking', 100.5)
        message.lost = True
        health.update('tracking', message, 100.5, 100.5)
        message.lost = False
        message.header.stamp.nanosec = 600000000
        health.update('tracking', message, 100.6, 100.6)
        self.assertIn('tracking', health.problem(100.6, 100.6))

    def test_old_replayed_data_does_not_count_as_fresh(self):
        health = ready_health()
        health.update('imu', message_for('imu', 90.0), 100.5, 100.5)
        self.assertIn('timestamp', health.problem(100.5, 100.5))

    def test_pose_jump_or_tracker_reset_latches_a_fault(self):
        for kind in ('lidar', 'filtered'):
            health = ready_health()
            message = message_for(kind, 100.5)
            message.pose.pose.position.x = 1.0
            health.update(kind, message, 100.5, 100.5)
            self.assertIn('jumped', health.problem(100.5, 100.5))

    def test_imu_without_orientation_is_usable_for_angular_velocity(self):
        self.assertIn('gyro', fused.sample('imu', message_for('imu')))

    def test_invalid_icp_pose_and_covariance_are_rejected(self):
        message = message_for('lidar')
        message.pose.covariance[0] = 9999.0
        with self.assertRaisesRegex(ValueError, 'covariance'):
            fused.sample('lidar', message)
        message = message_for('lidar')
        message.pose.pose.orientation.w = 0.0
        with self.assertRaisesRegex(ValueError, 'quaternion'):
            fused.sample('lidar', message)

    def test_health_failure_aborts_before_motion_publish(self):
        state = SimpleNamespace(health_problem=lambda: 'lidar: Sensor input stopped')
        with self.assertRaisesRegex(RuntimeError, 'lidar'):
            fused.FusedRectangle.tick(state, 0.1, 0.0)

    def test_observation_mode_rejects_nonzero_command(self):
        state = SimpleNamespace(health_problem=lambda: None, args=SimpleNamespace(observe=True))
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            fused.FusedRectangle.tick(state, 0.1, 0.0)


class MotionAgreementTests(unittest.TestCase):
    def states(self, poses):
        return {k: {'data': {'pose': list(p)}} for k, p in zip(
            ('wheel', 'lidar', 'filtered'), poses)}

    def test_opposite_forward_motion_aborts_before_publish(self):
        # 2026-09-08: wheel pose has a startup offset; LiDAR and EKF move backwards.
        initial = self.states([(-0.192, 0, 0), (0.001, 0, 0), (0.001, 0, 0)])
        current = self.states([(-0.142, 0, 0), (-0.037, 0, 0), (-0.035, 0, 0)])
        state = SimpleNamespace(health_problem=lambda: None,
            args=SimpleNamespace(observe=False), motion_check=fused.MotionAgreement(initial),
            health=SimpleNamespace(states=current), summary={}, phase='side_1',
            last_motion_log=0, record=lambda *a, **k: None)
        with patch.object(fused.Rectangle, 'tick') as publish:
            with self.assertRaisesRegex(RuntimeError, 'Wheel/LiDAR direction mismatch'):
                fused.FusedRectangle.tick(state, 0.05, 0)
            publish.assert_not_called()
        self.assertLess(state.summary['last_motion']['lidar']['forward_m'], 0)

    def test_matching_motion_in_all_four_headings_ignores_odom_origins(self):
        for heading in (0, math.pi/2, math.pi, -math.pi/2):
            initial = self.states([(2, -3, heading), (-1, 4, heading+0.1), (0, 0, heading)])
            check = fused.MotionAgreement(initial)
            current = self.states([s['data']['pose'] for s in initial.values()])
            for s in current.values():
                p = s['data']['pose']
                p[0] += 0.20*math.cos(p[2])
                p[1] += 0.20*math.sin(p[2])
            measured = check.measure(current)
            self.assertIsNone(check.problem(measured))
            for s in measured.values():
                self.assertAlmostEqual(s['forward_m'], 0.20)

    def test_startup_delay_noise_and_slip_do_not_claim_reversed_mount(self):
        check = fused.MotionAgreement(self.states([(0, 0, 0)]*3))
        for lidar_x in (0, -0.005, 0.03, 0.19):
            current = self.states([(0.20, 0, 0), (lidar_x, 0, 0), (lidar_x, 0, 0)])
            self.assertIsNone(check.problem(check.measure(current)))

    def test_turn_direction_handles_pi_crossing_and_detects_opposite_rotation(self):
        check = fused.MotionAgreement(self.states([(0, 0, 3.13)]*3), turning=True)
        current = self.states([(0, 0, fused.wrap(3.13+0.20))]*3)
        self.assertIsNone(check.problem(check.measure(current)))
        current['lidar']['data']['pose'][2] = 3.13-0.20
        self.assertIn('direction mismatch', check.problem(check.measure(current)))


if __name__ == '__main__':
    unittest.main()
