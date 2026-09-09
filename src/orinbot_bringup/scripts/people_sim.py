#!/usr/bin/env python3
"""Office world (office.sdf) dynamic pedestrian simulation node.

    ros2 run orinbot_bringup people_sim.py

Moves pedestrians back and forth along corridors in the office world to create dynamic obstacle scenarios.
Includes pause and turn-around behavior when robot approach is detected (within 1.4 m).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

ALIGN = math.radians(15)   # Angle tolerance to drive forward
REACH = 0.35               # Waypoint reach threshold [m]
RATE = 20.0

# Robot yield and avoidance parameters
YIELD_DIST = 1.4           # Detection range [m]
YIELD_FOV = math.radians(75)   # Horizontal FOV relative to heading
YIELD_CLEAR = 1.8          # Clearance distance to resume walking [m]
YIELD_GIVEUP = 12.0        # Yield timeout before turning around [s]

# (name, start_pose(x, y, yaw), waypoints, speed[m/s], turn[rad/s], pause_at_end[s])
ROUTES = [
    # Fast pace. Catches up from behind or passes in front.
    ('person_0', (-8.0, 0.0, 0.0), [(8.0, 0.0), (-8.0, 0.0)], 1.20, 1.6, 0.0),
    # Slow pace with long pause at end.
    ('person_1', (8.0, 6.0, math.pi), [(-8.0, 6.0), (8.0, 6.0)], 0.55, 0.9, 6.0),
    # Standard pace.
    ('person_2', (-8.0, -1.6, 0.0), [(8.0, -1.6), (-8.0, -1.6)], 0.85, 1.2, 2.0),
]


class Walker:

    def __init__(self, node, name, start, waypoints, speed, turn, pause):
        self.node = node
        self.name = name
        self.x, self.y, self.yaw = start
        self.pts = waypoints
        self.speed = speed
        self.w_turn = turn
        self.pause = pause

        self.i = 0
        self.rest = 0.0          # Remaining pause time [s]
        self.yielding = False    # Yielding to robot flag
        self.yield_t0 = None

        self.pub = node.create_publisher(Twist, '/model/%s/cmd_vel' % name, 10)

    def step(self, dt, robot_xy):
        t = Twist()
        if self._yield_check(robot_xy, dt):
            self.pub.publish(t)
            self._dead_reckon(t, dt)
            return

        if self.rest > 0:
            self.rest -= dt
            self.pub.publish(t)
            self._dead_reckon(t, dt)
            return

        tx, ty = self.pts[self.i]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        if dist < REACH:
            self.i = (self.i + 1) % len(self.pts)
            self.rest = self.pause
            self.pub.publish(t)
            self._dead_reckon(t, dt)
            return

        target_yaw = math.atan2(dy, dx)
        err = math.atan2(math.sin(target_yaw - self.yaw),
                         math.cos(target_yaw - self.yaw))

        if abs(err) > ALIGN:
            t.angular.z = self.w_turn if err > 0 else -self.w_turn
        else:
            t.linear.x = self.speed
            t.angular.z = max(-self.w_turn, min(self.w_turn, err * 2.0))

        self.pub.publish(t)
        self._dead_reckon(t, dt)

    def _yield_check(self, rx, dt):
        if rx is None:
            return False
        dx, dy = rx[0] - self.x, rx[1] - self.y
        dist = math.hypot(dx, dy)
        rel_angle = abs(math.atan2(math.sin(math.atan2(dy, dx) - self.yaw),
                                   math.cos(math.atan2(dy, dx) - self.yaw)))

        now = self.node.get_clock().now().nanoseconds * 1e-9

        if not self.yielding:
            if dist < YIELD_DIST and rel_angle < YIELD_FOV:
                self.yielding = True
                self.yield_t0 = now
                self.node.get_logger().info(
                    '%s yielding for robot (distance %.2f m)' % (self.name, dist))
                return True
            return False

        if dist > YIELD_CLEAR:
            self.yielding = False
            self.yield_t0 = None
            self.node.get_logger().info('%s resuming walk' % self.name)
            return False

        if self.yield_t0 and now - self.yield_t0 > YIELD_GIVEUP:
            self.yielding = False
            self.yield_t0 = None
            self.i = (self.i + 1) % len(self.pts)
            self.rest = 0.0
            self.node.get_logger().info(
                '%s yield timeout -- turning around to next waypoint' % self.name)
            return False

        return True

    def _dead_reckon(self, t, dt):
        self.yaw = math.atan2(math.sin(self.yaw + t.angular.z * dt),
                              math.cos(self.yaw + t.angular.z * dt))
        self.x += t.linear.x * math.cos(self.yaw) * dt
        self.y += t.linear.x * math.sin(self.yaw) * dt


class People(Node):

    def __init__(self):
        super().__init__('people_sim')
        p = self.declare_parameter
        p('robot_odom_topic', '/odom')
        odom_topic = self.get_parameter('robot_odom_topic').value

        self.robot = None
        self.create_subscription(Odometry, odom_topic, self._odom, 10)
        self.walkers = [Walker(self, *r) for r in ROUTES]
        self.create_timer(1.0 / RATE, self._tick)

    def _odom(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self):
        for wk in self.walkers:
            wk.step(1.0 / RATE, self.robot)


def main():
    rclpy.init()
    n = People()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
