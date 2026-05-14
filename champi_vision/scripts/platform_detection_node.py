#!/usr/bin/env python3
"""
NutBoxes Detection Node
Detects 4 kaplas (nutboxes) using ArUco tags on their surface.
Each kapla has an ArUco tag at its center: ID 36 = blue, ID 47 = yellow.
Uses marker size + camera intrinsics to estimate 3D pose (no depth image needed).
The node returns the center position of the 4-kapla group in base_link frame,
along with the color of each kapla sorted left-to-right.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge

from champi_interfaces.msg import NutBoxesDetection

from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import time

# ArUco ID to color mapping
ARUCO_ID_BLUE = 36
ARUCO_ID_YELLOW = 47

# Physical marker size in meters (side length of the ArUco tag)
MARKER_SIZE = 0.03  # 3cm — adjust to your actual tag size


def get_transfo_matrix(transform):
    rot = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y,
                       transform.transform.rotation.z, transform.transform.rotation.w])
    rot = rot.as_matrix()
    trans = np.array([transform.transform.translation.x, transform.transform.translation.y,
                      transform.transform.translation.z])
    transform_mtx = np.concatenate([rot, trans.reshape(3, 1)], axis=1)
    transform_mtx = np.concatenate([transform_mtx, np.array([[0, 0, 0, 1]])], axis=0)
    return transform_mtx


class NutBoxesDetectionNode(Node):
    def __init__(self):
        super().__init__('nut_boxes_detection_node')

        # CV Bridge
        self.cv_bridge = CvBridge()

        # RGB image
        self.latest_image = None
        self.image_subscriber = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        # Camera info (for intrinsics)
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.subscription_cam_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.callback_cam_info, 10)

        # ArUco detector
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform_matrix_cam_to_base_link = None

        # Publishers
        self.nutboxes_pub = self.create_publisher(NutBoxesDetection, '/nutboxes_detection', 10)
        self.info_image_pub = self.create_publisher(Image, '/nutboxes_detection_info', 10)

        # Timer (5 Hz)
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info("Nut Boxes Detection Node started (ArUco + PnP, no depth)!")

    # ================================================================
    # CALLBACKS
    # ================================================================

    def image_callback(self, msg):
        self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback_cam_info(self, msg):
        if self.camera_matrix is None:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics received.")

    # ================================================================
    # MAIN DETECTION LOOP
    # ================================================================

    def timer_callback(self):
        t_start = time.time()

        # Check prerequisites
        if self.latest_image is None or self.camera_matrix is None:
            return

        if self.transform_matrix_cam_to_base_link is None:
            if not self.init_transforms():
                return

        # Detect ArUco tags
        t_aruco_start = time.time()
        image = self.latest_image.copy()
        marker_corners, marker_ids, _ = self.aruco_detector.detectMarkers(image)
        t_aruco = time.time() - t_aruco_start

        # Build info image
        info_image = image.copy()

        if marker_ids is None or len(marker_ids) == 0:
            self.get_logger().debug("No ArUco tags detected")
            cv2.putText(info_image, "No ArUco detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self._publish_no_detection()
            t_pub_start = time.time()
            self._publish_info_image(info_image)
            t_pub = time.time() - t_pub_start
            t_total = time.time() - t_start
            # self.get_logger().info(f"⏱️ ArUco: {t_aruco*1000:.1f}ms | Publish: {t_pub*1000:.1f}ms | Total: {t_total*1000:.1f}ms")
            return

        # Filter for nutbox IDs only
        detections = []

        for i, marker_id in enumerate(marker_ids):
            mid = int(marker_id[0])
            if mid not in (ARUCO_ID_BLUE, ARUCO_ID_YELLOW):
                continue

            corners = marker_corners[i]

            # Estimate pose using solvePnP (marker size + intrinsics → 3D position in camera frame)
            obj_points = np.array([
                [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
                [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
                [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[0], self.camera_matrix, self.dist_coeffs)

            if not success:
                continue

            # tvec is position of marker in camera_color_optical_frame
            x_cam, y_cam, z_cam = float(tvec[0]), float(tvec[1]), float(tvec[2])

            # Transform to base_link
            pt_cam = np.array([x_cam, y_cam, z_cam, 1.0])
            pt_base = self.transform_matrix_cam_to_base_link @ pt_cam
            x_bl, y_bl, z_bl = float(pt_base[0]), float(pt_base[1]), float(pt_base[2])

            # Pixel center (for sorting and drawing)
            cx_px = int(np.mean(corners[0][:, 0]))
            cy_px = int(np.mean(corners[0][:, 1]))

            # Determine color
            if mid == ARUCO_ID_BLUE:
                color = NutBoxesDetection.COLOR_BLUE
                color_name = "BLUE"
                draw_color = (255, 0, 0)
            else:
                color = NutBoxesDetection.COLOR_YELLOW
                color_name = "YELLOW"
                draw_color = (0, 255, 255)

            detections.append((x_bl, y_bl, z_bl, color, color_name, draw_color, mid, cx_px, cy_px))

            # Draw axis on info image
            cv2.drawFrameAxes(info_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, MARKER_SIZE * 0.5)

        if len(detections) == 0:
            self.get_logger().debug("No nutbox tags (36/47) detected")
            cv2.putText(info_image, "No nutbox ArUco", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 128, 255), 2)
            self._publish_no_detection()
            t_pub_start = time.time()
            self._publish_info_image(info_image)
            t_pub = time.time() - t_pub_start
            t_total = time.time() - t_start
            # self.get_logger().info(f"⏱️ ArUco: {t_aruco*1000:.1f}ms | Publish: {t_pub*1000:.1f}ms | Total: {t_total*1000:.1f}ms")
            return

        # Sort by pixel x (left to right in image)
        detections.sort(key=lambda d: d[7])

        # # Compute center of the group in base_link
        # positions = np.array([(d[0], d[1]) for d in detections])
        # center_x = float(np.mean(positions[:, 0]))
        # center_y = float(np.mean(positions[:, 1]))

        # Instead, we get the leftmost position and we align the leftmost cup with it
        # Each box is 5cm wide, so we add 0.05m for each subsequent box to get the center of the group
        leftmost_x = detections[0][0]
        leftmost_y = detections[0][1]
        center_x = leftmost_x
        center_y = leftmost_y - 0.075
        # self.get_logger().info(f"##### Detected {len(detections)} tags | Center=({center_x:.3f}, {center_y:.3f})m | Colors={[d[4] for d in detections[:4]]}")
        # log leftmost
        # self.get_logger().info(f"Leftmost tag at ({leftmost_x:.3f}, {leftmost_y:.3f})m")

        # Fill colors (up to 4, sorted left-to-right)
        colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
        for i, det in enumerate(detections[:4]):
            colors[i] = det[3]

        # Draw annotations on info image
        for i, det in enumerate(detections[:4]):
            x_bl, y_bl, _, _, color_name, draw_color, aruco_id, px, py = det
            cv2.circle(info_image, (px, py), 12, draw_color, -1)
            label = f"#{aruco_id} {color_name}"
            cv2.putText(info_image, label, (px - 40, py - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
            cv2.putText(info_image, f"[{i}] ({x_bl:.2f},{y_bl:.2f})", (px - 50, py + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.putText(info_image, f"Center: ({center_x:.3f}, {center_y:.3f})m | {len(detections)} tags",
                    (10, info_image.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Publish result
        msg_out = NutBoxesDetection()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'base_link'
        msg_out.pose.position.x = center_x
        msg_out.pose.position.y = center_y
        msg_out.pose.position.z = 0.0
        msg_out.pose.orientation.w = 1.0
        msg_out.colors = colors
        self.nutboxes_pub.publish(msg_out)

        t_pub_start = time.time()
        self._publish_info_image(info_image)
        t_pub = time.time() - t_pub_start
        t_total = time.time() - t_start

        # self.get_logger().debug(
        #     f"✅ {len(detections)} tags | center=({center_x:.3f}, {center_y:.3f})m | "
        #     f"colors={[d[4] for d in detections[:4]]} | ⏱️ ArUco: {t_aruco*1000:.1f}ms | Publish: {t_pub*1000:.1f}ms | Total: {t_total*1000:.1f}ms"
        # )

    # ================================================================
    # HELPERS
    # ================================================================

    def _publish_no_detection(self):
        msg_out = NutBoxesDetection()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'base_link'
        msg_out.pose.position.z = -1.0  # sentinel: no detection
        msg_out.colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
        self.nutboxes_pub.publish(msg_out)

    def _publish_info_image(self, info_image):
        # Convert BGR to RGB for Foxglove compatibility
        info_image_rgb = cv2.cvtColor(info_image, cv2.COLOR_BGR2RGB)
        info_msg = self.cv_bridge.cv2_to_imgmsg(info_image_rgb, encoding='rgb8')
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_color_optical_frame'
        self.info_image_pub.publish(info_msg)

    def init_transforms(self):
        try:
            when = rclpy.time.Time().to_msg()
            transform_cam_to_base_link = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', when)
            self.transform_matrix_cam_to_base_link = get_transfo_matrix(transform_cam_to_base_link)
            self.get_logger().info("Transform camera_color_optical_frame -> base_link initialized.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform not initialized: {e}", throttle_duration_sec=5.0)
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = NutBoxesDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

