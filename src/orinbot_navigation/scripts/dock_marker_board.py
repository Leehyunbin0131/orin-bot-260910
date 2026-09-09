#!/usr/bin/env python3
"""ArUco marker board based dock pose estimation node.

    /camera/color/image_raw + camera_info  -->  /detected_dock_pose

Estimates dock pose from multiple ArUco markers (3 markers) using board-based pose estimation (estimatePoseBoard).
When approaching within lock_distance, locks the pose in fixed_frame to prevent close-range visual measurement divergence.
"""

import math

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (PoseStamped transform registration)
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def rvec_to_quat(rvec):
    """Convert rotation vector to quaternion (x, y, z, w)."""
    R, _ = cv2.Rodrigues(rvec)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


class DockMarkerBoard(Node):

    def __init__(self):
        super().__init__('dock_marker_board')
        p = self.declare_parameter

        # ArUco dictionary configuration (0 = DICT_4X4_50)
        p('dictionary', 0)
        # Marker IDs and X-axis offsets relative to dock center [m]
        p('marker_ids', [1, 0, 2])
        p('marker_dx', [-0.16, 0.0, 0.16])
        # Marker side length [m]
        p('marker_size', 0.10)
        # Minimum required detected markers
        p('min_markers', 2)

        # Close-range pose lock distance [m] (camera frame)
        p('lock_distance', 0.40)
        # Pose unlock distance [m]
        p('unlock_distance', 0.60)
        p('fixed_frame', 'odom')
        # Republish rate after pose lock [Hz]
        p('lock_publish_rate', 15.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        ids = [int(v) for v in g('marker_ids')]
        dxs = [float(v) for v in g('marker_dx')]
        self.msize = g('marker_size')
        self.min_markers = int(g('min_markers'))
        self.lock_d = g('lock_distance')
        self.unlock_d = g('unlock_distance')
        self.fixed_frame = g('fixed_frame')

        self.dict = cv2.aruco.Dictionary_get(int(g('dictionary')))
        self.params = cv2.aruco.DetectorParameters_create()
        # Subpixel refinement for marker corners
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.board = self._make_board(ids, dxs)
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.locked = None            # Locked PoseStamped (fixed_frame)

        # Power saving pause service handler
        self.paused = False
        self.create_service(Empty, '~/pause', self._srv_pause)
        self.create_service(Empty, '~/resume', self._srv_resume)

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(CameraInfo, 'camera_info', self._info, 10)
        self.create_subscription(Image, 'image', self._image, 10)
        self.create_timer(1.0 / g('lock_publish_rate'), self._republish)

        self.get_logger().info(
            'Dock marker board detection started: ids %s, offsets %s, size %.3f m / '
            'locking pose within %.2f m'
            % (ids, dxs, self.msize, self.lock_d))

    def _make_board(self, ids, dxs):
        """Construct 3D layout for 3 ArUco markers on board."""
        h = self.msize / 2.0
        obj = []
        for dx in dxs:
            ox = -dx
            obj.append(np.array([
                [ox - h,  h, 0.0],
                [ox + h,  h, 0.0],
                [ox + h, -h, 0.0],
                [ox - h, -h, 0.0]], dtype=np.float32))
        return cv2.aruco.Board_create(
            obj, self.dict, np.array(ids, dtype=np.int32).reshape(-1, 1))

    def _info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64).reshape(1, -1)

    def _srv_pause(self, _req, res):
        if not self.paused:
            self.paused = True
            self.get_logger().info('Marker detection paused')
        return res

    def _srv_resume(self, _req, res):
        if self.paused:
            self.paused = False
            self.locked = None
            self.get_logger().info('Marker detection resumed')
        return res

    def _image(self, msg):
        if self.paused or self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, self.dict, parameters=self.params)
        if ids is None or len(ids) < self.min_markers:
            return
        ok, rvec, tvec = cv2.aruco.estimatePoseBoard(
            corners, ids, self.board, self.K, self.D, None, None)
        if not ok:
            return
        dist = float(tvec[2])

        if self.locked is not None:
            if dist > self.unlock_d:
                self.get_logger().info('Distance %.2f m > unlock distance -- unlocking pose' % dist)
                self.locked = None
            else:
                return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = dist
        q = rvec_to_quat(rvec)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q

        if dist <= self.lock_d:
            latest = PoseStamped()
            latest.header.frame_id = pose.header.frame_id
            latest.pose = pose.pose
            try:
                self.locked = self.buf.transform(latest, self.fixed_frame)
                self.get_logger().info(
                    'Pose locked at %.2f m (%d markers detected)'
                    % (dist, len(ids)))
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warning('Pose transform lock failed: %s' % e)
                self.pub.publish(pose)
            return

        self.pub.publish(pose)

    def _republish(self):
        if self.paused:
            return
        if self.locked is None:
            return
        self.locked.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.locked)


def main():
    rclpy.init()
    node = DockMarkerBoard()
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
