"""Safety and signed-motion checks without opening hardware."""
import math
from pathlib import Path
import sys
import time
import unittest
from nav_msgs.msg import Odometry
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_service import MotionService, validate_move, wrap


class Harness:
    tick = MotionService.tick
    move = MotionService.move
    progress = MotionService.progress
    begin_stop = MotionService.begin_stop

    def __init__(self, kind='distance', target=0.1):
        self.pose = self.origin = (0., 0., 0.)
        self.velocity = (0., 0.)
        self.kind, self.target = kind, target
        self.state, self.reason = 'moving', ''
        self.deadline = time.monotonic()+100
        self.best, self.progress_time = abs(target), time.monotonic()
        self.odom_error = None
        self.last_odom = time.monotonic()
        self.command_id = 1
        self.issue = ''
        self.sent = []

    def problem(self): return self.issue
    def record(self, event): pass
    def send(self, linear=0., angular=0.): self.sent.append((linear, angular))


class MotionTests(unittest.TestCase):
    def test_units_and_invalid_requests(self):
        self.assertEqual(validate_move(-20, 0), ('distance', -.2))
        self.assertEqual(validate_move(0, -180), ('angle', -math.pi))
        for d, a in [(0, 0), (1, 1), (.5, 0), (201, 0), (0, 361), (math.nan, 0), (0, math.inf)]:
            with self.assertRaises(ValueError): validate_move(d, a)

    def test_reverse_and_right_turn(self):
        for kind, target, axis in [('distance', -.1, 0), ('angle', -.5, 1)]:
            h = Harness(kind, target); h.tick()
            self.assertLess(h.sent[-1][axis], 0)

    def test_progress_uses_initial_heading(self):
        h = Harness(); h.origin = (1., 1., math.pi/2); h.pose = (1., .8, math.pi/2)
        self.assertAlmostEqual(h.progress()[0], -.2)
        self.assertAlmostEqual(wrap(math.radians(-358)), math.radians(2))

    def test_busy_and_unready_reject(self):
        h = Harness(); req = SimpleNamespace(distance_cm=10., angle_deg=0.)
        self.assertFalse(h.move(req, SimpleNamespace()).accepted)
        h.state, h.issue = 'idle', 'Odometry missing'
        self.assertFalse(h.move(req, SimpleNamespace()).accepted)
        self.assertEqual(h.sent, [])

    def test_feedback_loss_and_publisher_conflict_stop(self):
        for issue in ['Odometry missing', 'Another publisher', 'Odometry jumped']:
            h = Harness(); h.issue = issue; h.tick()
            self.assertEqual(h.state, 'stopping'); self.assertEqual(h.result_state, 'failed')
            self.assertEqual(h.sent[-1], (0, 0))

    def test_stall_and_timeout(self):
        for attr in ['deadline', 'progress_time']:
            h = Harness(); setattr(h, attr, time.monotonic()-10); h.tick()
            self.assertEqual(h.result_state, 'failed'); self.assertEqual(h.sent[-1], (0, 0))

    def test_settle_requires_fresh_stationary_feedback(self):
        h = Harness(); h.pose = (.1, 0, 0); h.tick()
        self.assertEqual(h.state, 'stopping')
        h.stable = time.monotonic()-1; h.velocity = (.01, 0); h.tick()
        self.assertEqual(h.state, 'stopping'); self.assertIsNone(h.stable)
        h.velocity = (0, 0); h.stable = time.monotonic()-1; h.tick()
        self.assertEqual(h.state, 'succeeded')

    def test_odometry_unwrap_and_reject_bad_feedback(self):
        def sample(yaw, stamp):
            m = Odometry(); m.header.frame_id = 'odom'; m.child_frame_id = 'base_footprint'
            m.header.stamp.sec = stamp
            m.pose.pose.orientation.z = math.sin(yaw/2)
            m.pose.pose.orientation.w = math.cos(yaw/2)
            return m
        h = Harness(); h.pose = (0., 0., math.radians(179)); h.last_stamp = 9*10**9
        h.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10*10**9))
        m = sample(math.radians(-179), 10)
        MotionService.odom_callback(h, m)
        self.assertAlmostEqual(h.pose[2], math.radians(181))
        MotionService.odom_callback(h, m)
        self.assertIn('timestamp', h.odom_error)
        h.odom_error = None; m.pose.pose.orientation.w = math.nan
        MotionService.odom_callback(h, m)
        self.assertEqual(h.odom_error, 'Invalid odometry values')

    def test_stop_timeout_and_overshoot_fail(self):
        h = Harness(); h.begin_stop('succeeded', ''); h.last_odom = 0; h.stop_deadline = 0; h.tick()
        self.assertEqual(h.state, 'failed')
        h = Harness(); h.begin_stop('succeeded', ''); h.pose = (.12, 0, 0); h.stable = time.monotonic()-1; h.tick()
        self.assertEqual(h.state, 'failed')


if __name__ == '__main__': unittest.main()
