#!/usr/bin/env python3
"""Node that periodically republishes visual odometry correction TF (vodom -> odom) at a constant rate.

Computes T(vodom->odom) = T(vodom->base) * T(odom->base)^-1 to prevent Nav2 TF lookup timeouts
caused by processing delays in visual odometry.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def q_mult(a, b):
    """Quaternion multiplication (x, y, z, w order)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_rotate(q, v):
    """Rotate vector v by quaternion q."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    return q_mult(q_mult(q, qv), q_conj(q))[:3]


class VodomTfRelay(Node):

    def __init__(self):
        super().__init__('vodom_tf_relay')

        self.declare_parameter('visual_odom_topic', '/vodom')
        self.declare_parameter('visual_odom_frame', 'vodom')
        self.declare_parameter('wheel_odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)
        # Timestamp prediction offset [s] to prevent future extrapolation errors during TF lookup
        self.declare_parameter('tf_tolerance', 0.2)

        self.visual_frame = self.get_parameter('visual_odom_frame').value
        self.wheel_frame = self.get_parameter('wheel_odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_tolerance = float(self.get_parameter('tf_tolerance').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # Initial identity transform to maintain TF chain
        self.correction_t = np.zeros(3)
        self.correction_q = np.array([0.0, 0.0, 0.0, 1.0])
        self.have_correction = False

        # 도킹처럼 카메라가 벽을 코앞에서 보는 구간에서는 시각 오도메트리가
        # 깨지므로 보정을 얼립니다. 얼린 동안에도 TF 는 계속 나가야 합니다 —
        # 끊으면 map 과 base_footprint 가 다른 트리로 갈라집니다.
        # 보정 자체가 천천히 변하는 값이라 짧은 구간은 휠+IMU 로 충분합니다.
        self.frozen = False
        self.create_service(Empty, '~/pause', self._srv_pause)
        self.create_service(Empty, '~/resume', self._srv_resume)

        self.create_subscription(
            Odometry, self.get_parameter('visual_odom_topic').value,
            self.on_visual_odom, 10)
        self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            'Publishing %s -> %s correction TF at %.0f Hz with %+.2fs offset.'
            % (self.visual_frame, self.wheel_frame, rate, self.tf_tolerance))

    # ------------------------------------------------------------------
    def _srv_pause(self, _req, res):
        if not self.frozen:
            self.frozen = True
            self.get_logger().info('시각 오도메트리 보정 동결')
        return res

    def _srv_resume(self, _req, res):
        if self.frozen:
            self.frozen = False
            self.get_logger().info('시각 오도메트리 보정 재개')
        return res

    def on_visual_odom(self, msg: Odometry):
        """Update T(vodom->odom) = T(vodom->base) * T(odom->base)^-1."""
        if self.frozen:
            return
        try:
            wheel = self.buffer.lookup_transform(
                self.wheel_frame, self.base_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except Exception as exc:  # TF lookup unavailable or timestamp mismatch
            self.get_logger().warn(
                'Wheel odometry TF lookup failed: %s' % exc, throttle_duration_sec=5.0)
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t1 = np.array([p.x, p.y, p.z])
        q1 = np.array([o.x, o.y, o.z, o.w])

        wt = wheel.transform.translation
        wr = wheel.transform.rotation
        t2 = np.array([wt.x, wt.y, wt.z])
        q2 = np.array([wr.x, wr.y, wr.z, wr.w])

        q = q_mult(q1, q_conj(q2))
        t = t1 - q_rotate(q, t2)

        self.correction_q = q / np.linalg.norm(q)
        self.correction_t = t
        self.have_correction = True

    # ------------------------------------------------------------------
    def publish(self):
        stamp = self.get_clock().now() + Duration(seconds=self.tf_tolerance)

        msg = TransformStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.visual_frame
        msg.child_frame_id = self.wheel_frame
        msg.transform.translation.x = float(self.correction_t[0])
        msg.transform.translation.y = float(self.correction_t[1])
        msg.transform.translation.z = float(self.correction_t[2])
        msg.transform.rotation.x = float(self.correction_q[0])
        msg.transform.rotation.y = float(self.correction_q[1])
        msg.transform.rotation.z = float(self.correction_q[2])
        msg.transform.rotation.w = float(self.correction_q[3])
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = VodomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
