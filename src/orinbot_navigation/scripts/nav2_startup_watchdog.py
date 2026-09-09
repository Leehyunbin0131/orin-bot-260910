#!/usr/bin/env python3
"""Nav2 lifecycle startup monitoring and auto-retry watchdog node.

Monitors Nav2 bringup state and invokes lifecycle_manager's manage_nodes service to RESET then STARTUP
if node activation fails. Terminates cleanly once all target nodes are verified active.
"""

import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_MANAGER = '/lifecycle_manager_navigation'


class Watchdog(Node):

    def __init__(self):
        super().__init__('nav2_startup_watchdog')
        p = self.declare_parameter
        # Target lifecycle_manager to monitor
        p('manager', DEFAULT_MANAGER)
        # Timeout waiting for initial startup [s]
        p('startup_timeout', 60.0)
        # Timeout waiting for retry startup [s]
        p('retry_timeout', 45.0)
        p('max_retries', 3)
        p('poll_period', 2.0)

        g = self.get_parameter
        self.startup_timeout = g('startup_timeout').value
        self.retry_timeout = g('retry_timeout').value
        self.max_retries = int(g('max_retries').value)
        self.period = g('poll_period').value

        self.manager = g('manager').value
        self.is_active = self.create_client(Trigger, self.manager + '/is_active')
        self.manage = self.create_client(
            ManageLifecycleNodes, self.manager + '/manage_nodes')

    def call(self, client, req, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def active(self):
        res = self.call(self.is_active, Trigger.Request(), timeout=3.0)
        return res is not None and res.success

    def restart(self):
        # RESET must be called before STARTUP if some nodes are already partially active
        self.call(self.manage, ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.RESET), timeout=30.0)
        self.call(self.manage, ManageLifecycleNodes.Request(
            command=ManageLifecycleNodes.Request.STARTUP), timeout=60.0)

    def run(self):
        self.get_logger().info('Starting activation watchdog for %s' % self.manager)
        if not self.is_active.wait_for_service(timeout_sec=120.0):
            self.get_logger().error('Service %s unavailable' % self.manager)
            return 1

        deadline = time.monotonic() + self.startup_timeout
        retries = 0
        while rclpy.ok():
            if self.active():
                self.get_logger().info(
                    '%s activation verified (retries: %d) -- exiting watchdog'
                    % (self.manager, retries))
                return 0
            if time.monotonic() < deadline:
                time.sleep(self.period)
                continue
            if retries >= self.max_retries:
                self.get_logger().error(
                    '%s activation failed after %d retries. Check for leftover processes or multiple /clock publishers.'
                    % (self.manager, retries))
                return 1
            retries += 1
            self.get_logger().warn(
                '%s activation failed -- retrying RESET then STARTUP (%d/%d)'
                % (self.manager, retries, self.max_retries))
            self.restart()
            deadline = time.monotonic() + self.retry_timeout
        return 0


def main():
    rclpy.init()
    node = Watchdog()
    code = node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
