#!/usr/bin/env python3
"""Battery state simulator node (sensor_msgs/BatteryState).

Simulates battery discharge, charging, and Gazebo ground truth dock contact evaluation.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class BatterySim(Node):

    def __init__(self):
        super().__init__('battery_sim')
        p = self.declare_parameter

        p('publish_rate', 1.0)
        p('capacity_ah', 10.0)          # 24 V 10 Ah = 240 Wh
        p('voltage_full', 29.4)         # 7S Li-ion fully charged
        p('voltage_empty', 21.0)        # Discharge cutoff
        p('initial_soc', 0.85)

        # Current draw parameters [A]
        p('idle_current', 1.25)              # Compute + Sensors (~30 W)
        p('current_per_mps', 3.0)            # Linear velocity proportional current
        p('current_per_radps', 0.6)          # Angular velocity proportional current
        p('charge_current', 3.0)             # Charge current (~3.3 hours)
        p('taper_from_soc', 0.95)            # Current taper start SOC

        # Simulation speedup factor
        p('speedup', 1.0)

        # Target dock pose (must match home_dock.pose in docking.yaml)
        p('dock_x', 1.0)
        p('dock_y', -3.67)
        # **도킹 완료 시 로봇이 향하는 방향**입니다.
        # 후진 도킹(reverse_dock)으로 바꾸면 +1.5708 로 뒤집어야 합니다.
        p('dock_yaw', 1.5708)
        # Contact tolerances
        p('contact_tolerance_lon', 0.048)     # Longitudinal tolerance [m]
        p('contact_tolerance_lat', 0.034)     # Lateral tolerance [m]
        p('contact_tolerance_yaw', 0.11)      # Yaw tolerance [rad] (~6.3 deg)
        p('ground_truth_topic', '/ground_truth/odom')
        p('base_frame', 'base_footprint')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cap_ah = g('capacity_ah')
        self.v_full, self.v_empty = g('voltage_full'), g('voltage_empty')
        self.i_idle = g('idle_current')
        self.i_mps, self.i_radps = g('current_per_mps'), g('current_per_radps')
        self.i_charge = g('charge_current')
        self.taper = g('taper_from_soc')
        self.speedup = g('speedup')
        self.dock = (g('dock_x'), g('dock_y'), g('dock_yaw'))
        self.tol_lon = g('contact_tolerance_lon')
        self.tol_lat = g('contact_tolerance_lat')
        self.tol_yaw = g('contact_tolerance_yaw')
        self.base_frame = g('base_frame')

        self.soc = max(0.0, min(1.0, g('initial_soc')))
        self.v, self.w = 0.0, 0.0
        self.last = None
        self.truth = None            # (x, y, yaw)
        self.warned_no_truth = False

        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)
        self.create_subscription(Odometry, g('ground_truth_topic'),
                                 self._truth, 10)
        self.create_subscription(Float32, '~/set_soc', self._set_soc, 1)
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)

        self.create_timer(1.0 / g('publish_rate'), self._tick)
        self.get_logger().info(
            'Battery simulator started: %.0f%% / %.1f Ah / speedup %.0fx / '
            'dock (%.2f, %.2f) / tolerance lon %.0fmm lat %.0fmm yaw %.1f deg'
            % (self.soc * 100, self.cap_ah, self.speedup, self.dock[0],
               self.dock[1], self.tol_lon * 1000, self.tol_lat * 1000,
               math.degrees(self.tol_yaw)))

    def _odom(self, m):
        self.v = m.twist.twist.linear.x
        self.w = m.twist.twist.angular.z

    def _set_soc(self, m):
        self.soc = max(0.0, min(1.0, float(m.data)))
        self.get_logger().warning('Forced battery SOC to %.0f%%' % (self.soc * 100))

    def _truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y,
                      math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _on_dock(self):
        if self.truth is None:
            if not self.warned_no_truth:
                self.warned_no_truth = True
                self.get_logger().warning(
                    'Ground truth pose (%s) unavailable -- cannot evaluate docking contact.'
                    % self.get_parameter('ground_truth_topic').value)
            return False
        x, y, yaw = self.truth
        dx, dy = x - self.dock[0], y - self.dock[1]
        c, s = math.cos(self.dock[2]), math.sin(self.dock[2])
        lon = dx * c + dy * s
        lat = -dx * s + dy * c
        dyaw = math.atan2(math.sin(yaw - self.dock[2]),
                          math.cos(yaw - self.dock[2]))
        return (abs(lon) <= self.tol_lon and abs(lat) <= self.tol_lat
                and abs(dyaw) <= self.tol_yaw)

    def _tick(self):
        now = self.get_clock().now()
        if self.last is None:
            self.last = now
            return
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now

        dt_sim = dt * self.speedup
        docked = self._on_dock()

        if docked and self.soc < 1.0:
            if self.soc >= self.taper:
                frac = (1.0 - self.soc) / (1.0 - self.taper)
                current = self.i_charge * max(0.1, frac)
            else:
                current = self.i_charge
            delta_ah = (current * dt_sim) / 3600.0
            self.soc = min(1.0, self.soc + delta_ah / self.cap_ah)
            is_charging = True
        else:
            current = self.i_idle + abs(self.v) * self.i_mps + abs(self.w) * self.i_radps
            delta_ah = (current * dt_sim) / 3600.0
            self.soc = max(0.0, self.soc - delta_ah / self.cap_ah)
            is_charging = False

        m = BatteryState()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.base_frame
        m.voltage = float(self.v_empty + (self.v_full - self.v_empty) * self.soc)
        m.temperature = 25.0
        m.current = float(current if is_charging else -current)
        m.charge = float(self.cap_ah * self.soc)
        m.capacity = float(self.cap_ah)
        m.design_capacity = float(self.cap_ah)
        m.percentage = float(self.soc)

        if is_charging:
            m.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_FULL
                if self.soc >= 0.999
                else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        m.present = True
        self.pub.publish(m)


def main():
    rclpy.init()
    n = BatterySim()
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
