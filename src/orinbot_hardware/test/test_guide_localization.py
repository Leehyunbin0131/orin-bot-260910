"""Delayed initial pose must not consume Nav2's activation timeout."""
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from concurrent.futures import Future

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from guide_service import GuideService
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes


class LocalizationTests(unittest.TestCase):
    def node(self):
        now=time.monotonic()
        node=NS(drive=True,selected=object(),process=NS(poll=lambda:None),navigation_fault=False,
                nav_start_future=None,nav_start_error='',lock=threading.RLock(),
                lifecycle={n:(3,now) for n in ('map_server','amcl')},
                last_initial=0,initial_started=now-70,initialized=False,amcl=None,
                input_problem=lambda:'',event=Mock(),amcl_updates=0,last_amcl=0,
                tf=NS(lookup_transform=lambda *args:NS(header=NS(stamp=None))),fresh_stamp=lambda stamp:True,
                count_publishers=lambda topic:1,action=NS(server_is_ready=lambda:True))
        node.localization_problem=lambda:GuideService.localization_problem(node)
        node.nav_start=Mock();node.nav_start.service_is_ready.return_value=True
        node.nav_start.call_async.return_value=Future()
        return node

    def localized(self,node):
        node.last_initial=1
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp.sec=1
        msg.pose.covariance[0]=msg.pose.covariance[7]=msg.pose.covariance[35]=.04
        GuideService.amcl_callback(node,msg)

    def test_long_wait_does_not_start_navigation_until_pose_arrives(self):
        n=self.node()
        self.assertIn('현재 위치',GuideService.ready_problem(n))
        GuideService.start_navigation_if_localized(n)
        n.nav_start.call_async.assert_not_called()
        n.last_initial=1
        self.assertIn('70초',GuideService.ready_problem(n))
        GuideService.start_navigation_if_localized(n)
        n.nav_start.call_async.assert_not_called()
        self.localized(n)
        GuideService.start_navigation_if_localized(n)
        GuideService.start_navigation_if_localized(n)
        n.nav_start.call_async.assert_called_once()
        self.assertEqual(n.nav_start.call_async.call_args.args[0].command,ManageLifecycleNodes.Request.STARTUP)
        n.nav_start_future.set_result(NS(success=True))
        self.assertIn('주행 서버 준비',GuideService.ready_problem(n))
        for name in ('planner_server','controller_server','bt_navigator'):n.lifecycle[name]=(3,time.monotonic())
        self.assertEqual(GuideService.ready_problem(n),'')

    def test_old_wrong_frame_or_uncertain_pose_cannot_enable_navigation(self):
        n=self.node();n.last_initial=2*10**9
        for stamp,frame in ((1,'map'),(3,'odom')):
            msg=PoseWithCovarianceStamped();msg.header.stamp.sec=stamp;msg.header.frame_id=frame
            GuideService.amcl_callback(n,msg)
        self.assertFalse(n.initialized)
        self.localized(n)
        for value in (math.nan,-1.,.26):
            n.amcl.pose.covariance[0]=value
            GuideService.start_navigation_if_localized(n)
            n.nav_start.call_async.assert_not_called()
        n.amcl.pose.covariance[0]=.04;n.input_problem=lambda:'Fresh /scan is missing'
        GuideService.start_navigation_if_localized(n)
        n.nav_start.call_async.assert_not_called()

    def test_start_failure_is_reported_and_old_map_callback_is_ignored(self):
        n=self.node();self.localized(n)
        GuideService.start_navigation_if_localized(n)
        n.nav_start_future.set_result(NS(success=False))
        self.assertIn('시작하지 못했습니다',GuideService.ready_problem(n))
        n=self.node();self.localized(n)
        GuideService.start_navigation_if_localized(n)
        old=n.nav_start_future;n.nav_start_future=None
        old.set_result(NS(success=False))
        self.assertEqual(n.nav_start_error,'')


if __name__=='__main__':unittest.main()
