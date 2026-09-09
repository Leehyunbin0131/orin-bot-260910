import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location(
    'rectangle_example', Path(__file__).resolve().parents[1]/'scripts/drive_rectangle.py')
rectangle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rectangle)


class RectangleTests(unittest.TestCase):
    def test_heading_correction_never_exceeds_requested_wheel_rpm(self):
        limit = 218*rectangle.RAW_UNIT
        for linear, angular in ((0.17, 0.13), (-0.17, -0.13), (0, 1), (0, 0)):
            v, w = rectangle.bounded_twist(linear, angular, 0.0325, 0.5, limit)
            self.assertLessEqual(max(abs(v-w*0.25), abs(v+w*0.25))/0.0325, limit+1e-9)
            if linear and angular:
                self.assertAlmostEqual(v/w, linear/angular)

    def test_turn_angle_continues_across_pi_boundary(self):
        state = SimpleNamespace(pose=(0, 0, 3.13), last_stamp=1, odom_error=None,
                                last_log=0, last_command=(0, 0), record=lambda *a, **k: None)
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_footprint'
        message.header.stamp.sec = 1
        message.pose.pose.orientation.z = math.sin(-3.13/2)
        message.pose.pose.orientation.w = math.cos(-3.13/2)
        rectangle.Rectangle.odom_callback(state, message)
        self.assertGreater(state.pose[2], math.pi)
        self.assertAlmostEqual(state.pose[2], 2*math.pi-3.13)
        self.assertIsNone(state.odom_error)

    def test_lost_odometry_aborts_before_sending_motion(self):
        state = SimpleNamespace(odom_error=None, last_odom=time.monotonic()-1)
        with self.assertRaisesRegex(RuntimeError, 'Odometry missing'):
            rectangle.Rectangle.tick(state, 0.17, 0.0)

    def test_heading_correction_at_60rpm_keeps_each_commanded_wheel_bounded(self):
        limit = 262*rectangle.RAW_UNIT
        for v in (-0.204, 0, 0.204):
            for w in (-0.340, 0, 0.340):
                linear, angular = rectangle.bounded_twist(v, w, 0.0325, 0.50, limit)
                self.assertLessEqual(max(abs(linear-angular*0.25),
                                         abs(linear+angular*0.25))/0.0325, limit+1e-9)

    def simulate(self, turn_rpm, whole_rectangle=False):
        # Ideal differential drive with the controller's acceleration limits.
        # Virtual time only: no ROS nodes, serial ports or physical motor commands.
        now = [1.0]
        limit = 262*rectangle.RAW_UNIT
        scale = max(1.0, turn_rpm/10)
        state = SimpleNamespace(pose=(0., 0., 0.), velocity=(0., 0.),
            speed=limit*0.0325, turn_speed=2*math.floor(turn_rpm/0.229)*rectangle.RAW_UNIT*0.0325/0.50,
            turn_response_scale=scale, turn_rpm=turn_rpm, phase='waiting',
            args=SimpleNamespace(width=1., height=1., rpm=60.), summary={'corners': []},
            wait_ready=lambda: None, record=lambda *a, **k: None)

        def tick(linear=0., angular=0.):
            linear, angular = rectangle.bounded_twist(linear, angular, 0.0325, 0.50, limit)
            v, w = state.velocity
            v += max(-0.05*0.05, min(0.05*0.05, linear-v))
            w += max(-0.10*scale*0.05, min(0.10*scale*0.05, angular-w))
            x, y, yaw = state.pose
            state.pose = (x+v*math.cos(yaw+w*0.025)*0.05,
                          y+v*math.sin(yaw+w*0.025)*0.05, yaw+w*0.05)
            state.velocity = (v, w)
            now[0] += 0.05

        state.tick = tick
        state.settle = lambda: rectangle.Rectangle.settle(state)
        state.segment = lambda *a, **k: rectangle.Rectangle.segment(state, *a, **k)
        with patch.object(rectangle.time, 'monotonic', side_effect=lambda: now[0]):
            if whole_rectangle:
                rectangle.Rectangle.run(state)
            else:
                state.segment(0, math.pi/2, turning=True)
        return state, now[0]-1

    def test_1m_rectangle_completes_with_acceleration_limits_at_60rpm(self):
        state, _ = self.simulate(25, whole_rectangle=True)
        self.assertEqual(state.summary['status'], 'pass')
        self.assertEqual(len(state.summary['corners']), 4)
        self.assertLess(state.summary['closure_error_m'], 0.10)
        self.assertAlmostEqual(state.pose[2], 2*math.pi, delta=0.03)
        self.assertLess(abs(state.velocity[0]), 0.003)
        self.assertLess(abs(state.velocity[1]), 0.01)

    def test_25rpm_turn_finishes_in_less_than_half_the_previous_time(self):
        _, old_duration = self.simulate(10)
        _, new_duration = self.simulate(25)
        self.assertLess(new_duration, old_duration/2)

    def test_stop_publication_covers_60rpm_deceleration_and_feedback(self):
        speed = 262*rectangle.RAW_UNIT*0.0325
        self.assertGreater(rectangle.STOP_PUBLISH_SECONDS, speed/0.05+0.3+0.5)


if __name__ == '__main__':
    unittest.main()
