"""Navigation cruise speed and combined wheel-command bounds without hardware."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from guide_service import GuideService
from builtin_interfaces.msg import Time
spec=importlib.util.spec_from_file_location('base_speed',ROOT/'launch/base.launch.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)


class GuideSpeedTests(unittest.TestCase):
    def test_nav2_and_controller_match_60rpm_geometry(self):
        settings=yaml.safe_load((ROOT/'config/hardware.yaml').read_text())
        params=yaml.safe_load((ROOT/'config/controllers.yaml').read_text())['diff_drive_controller']['ros__parameters']
        nav=yaml.safe_load((ROOT/'config/guide_nav.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
        base.configure_guide_profile(settings,params)
        raw_limit=math.floor(60/.229)*.0239691227
        expected=raw_limit*settings['wheel_radius']
        self.assertAlmostEqual(nav['desired_linear_vel'],expected)
        self.assertAlmostEqual(params['linear.x.max_velocity'],expected)
        self.assertAlmostEqual(nav['rotate_to_heading_angular_vel'],params['angular.z.max_velocity'])
        for axis in ('linear.x','angular.z'):
            self.assertAlmostEqual(params[f'{axis}.max_acceleration']*4,params[f'{axis}.max_velocity'])
            self.assertAlmostEqual(-params[f'{axis}.max_deceleration'],params[f'{axis}.max_velocity'])
        self.assertTrue(nav['use_collision_detection'])
        self.assertTrue(nav['use_regulated_linear_velocity_scaling'])
        self.assertFalse(nav['allow_reversing'])
        self.assertLess(nav['min_approach_linear_velocity'],nav['desired_linear_vel'])

    def test_gate_allows_cruise_and_rotation_but_bounds_combined_commands(self):
        output=[]
        n=NS(radius=.0325,track=.44,wheel_limit=262*.0239691227,
             velocity_pub=NS(publish=output.append),
             get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time())))
        v=n.wheel_limit*n.radius;w=2*v/n.track
        for linear in (-1.,0.,v/2,v,1.):
            for angular in (-10.,-w,0.,w,10.):
                GuideService.send(n,linear,angular)
                command=output[-1].twist
                self.assertGreaterEqual(command.linear.x,0)
                wheels=[(command.linear.x+sign*command.angular.z*n.track/2)/n.radius for sign in (-1,1)]
                self.assertLessEqual(max(map(abs,wheels)),n.wheel_limit+1e-9)
        GuideService.send(n,v,0)
        self.assertAlmostEqual(output[-1].twist.linear.x,v)
        GuideService.send(n,0,w)
        self.assertAlmostEqual(output[-1].twist.angular.z,w)
        GuideService.send(n)
        self.assertEqual((output[-1].twist.linear.x,output[-1].twist.angular.z),(0,0))


if __name__=='__main__':unittest.main()
