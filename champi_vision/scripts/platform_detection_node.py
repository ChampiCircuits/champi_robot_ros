#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from champi_interfaces.msg import NutBoxesDetection
from champi_vision.aruco_localizer import ArucoDetector

from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
from icecream import ic

# ArUco ID to color mapping
ARUCO_ID_BLUE = 36
ARUCO_ID_YELLOW = 47

def create_point_cloud2(points_array, frame_id):
    """
    Convertit un tableau Nx3 numpy array ou liste de points [x, y, z] en PointCloud2.
    """
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = frame_id

    # Création de la liste de tuples (x, y, z)
    points = [tuple(p) for p in points_array]

    # Générer le message PointCloud2
    cloud_msg = create_cloud_xyz32(header, points)
    return cloud_msg

def get_transfo_matrix(transform):
    #get the rotation and translation matrix
    rot = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
    rot = rot.as_matrix()
    trans = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

    # create the transformation matrix
    transform_mtx = np.concatenate([rot, trans.reshape(3,1)], axis=1)
    transform_mtx = np.concatenate([transform_mtx, np.array([[0,0,0,1]])], axis=0)

    return transform_mtx


def transform_points_array_to_frame_with_transform_matrix(points, transform_matrix):
    # Step 1: Convert to homogeneous coordinates (N,3) → (N,4)
    ones = np.ones((points.shape[0], 1))  # Shape: (N,1)

    points_homogeneous = np.concatenate([points, ones], axis=1)

    # Step 2: Apply the transformation
    transformed_points_homogeneous = points_homogeneous @ transform_matrix.T  # Matrix multiplication

    # Step 3: Convert back to (N,3) by removing the homogeneous coordinate
    transformed_points = transformed_points_homogeneous[:, :3]

    return transformed_points


def find_box_pose_2d(points_3d):
    """
    Fit an oriented bounding rectangle to the 2D (x, y) projection of top-plane points.
    Returns (cx, cy, angle_rad, long_side_m, short_side_m) in base_link frame.
    angle_rad is the orientation of the long axis of the box.
    Requires at least 5 points.
    """
    pts_2d = points_3d[:, :2].astype(np.float32).reshape((-1, 1, 2))
    (cx, cy), (w, h), angle_deg = cv2.minAreaRect(pts_2d)
    # Normalise: long_side >= short_side, angle refers to the long axis
    if w < h:
        angle_deg += 90.0
        w, h = h, w
    return float(cx), float(cy), float(np.deg2rad(angle_deg)), float(w), float(h)


class NutBoxesDetectionNode(Node):
    def __init__(self):
        super().__init__('nut_boxes_detection_node')

        # point cloud sub
        self.point_cloud_sub = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.point_cloud_callback, 10)
        self.latest_point_cloud = None

        # RGB image sub
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        self.subscription_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.callback_cam_info,
            10)
        self.camera_info = None

        # ArUco detector
        self.aruco_detector = ArucoDetector()

        # info image publisher
        self.info_image_pub = self.create_publisher(Image, '/nutboxes_detection_info', 10)

        # rviz filtered point cloud pub
        self.filtered_point_cloud_pub = self.create_publisher(PointCloud2, '/rviz/filtered_point_cloud', 10)

        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform_matrix_cam_to_base_link = None
        self.transform_matrix_base_link_to_odom = None

        # Nuts Box to detect: dimensions (length x width x height in meters)
        self.box_length = 0.150
        self.box_width  = 0.050 * 4  # we want to detect the whole cluster of 4 boxes, not each box separately, so we consider a "virtual box" that encompasses the 4 boxes
        self.box_height = 0.030
        height_margin = 0.02  # m, tolerance on z filtering
        self.top_plane_height_interval = [self.box_height - height_margin, self.box_height + height_margin]

        # distance publisher
        self.nutboxes_relative_position_pub = self.create_publisher(NutBoxesDetection, '/nutboxes_detection', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Nut Boxes Detection Node started !")

    def image_callback(self, msg):
        """Store latest RGB image."""
        self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback_cam_info(self, msg):
        """Store camera info."""
        self.camera_info = msg

    def detect_nutbox_colors(self):
        """Detect ArUco tags on the latest RGB image and return colors sorted left-to-right.

        The robot approaches perpendicular to the 4 kaplas, so the kaplas are
        side-by-side along the image x-axis. Sorting detected tags by their
        x-pixel coordinate gives left-to-right order.

        Returns:
            (colors, info_image): list of 4 color constants, and annotated BGR image (or None).
        """
        colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
        info_image = None

        if self.latest_image is None:
            return colors, info_image

        image = self.latest_image.copy()
        info_image = image.copy()

        # Detect only nutbox ArUco IDs (36=blue, 47=yellow)
        poses, ids = self.aruco_detector.detect_arucos(image, ids_to_find=[ARUCO_ID_BLUE, ARUCO_ID_YELLOW])

        if len(ids) == 0:
            self.get_logger().info("No nutbox ArUco tags detected")
            cv2.putText(info_image, "No ArUco detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return colors, info_image

        # Build list of (center_x_pixel, center_y_pixel, color, color_name, draw_color, aruco_id)
        detections = []
        for pose, aruco_id in zip(poses, ids):
            cx_px, cy_px = pose[0], pose[1]
            if aruco_id == ARUCO_ID_BLUE:
                color = NutBoxesDetection.COLOR_BLUE
                color_name = "BLUE"
                draw_color = (255, 0, 0)  # BGR
            elif aruco_id == ARUCO_ID_YELLOW:
                color = NutBoxesDetection.COLOR_YELLOW
                color_name = "YELLOW"
                draw_color = (0, 255, 255)  # BGR
            else:
                continue
            detections.append((cx_px, cy_px, color, color_name, draw_color, aruco_id))

        # Sort by x pixel coordinate (left to right in the image)
        detections.sort(key=lambda d: d[0])

        # Fill colors array (up to 4)
        for i, det in enumerate(detections[:4]):
            colors[i] = det[2]

        # Draw annotations on info image
        for i, det in enumerate(detections[:4]):
            cx_px, cy_px, _, color_name, draw_color, aruco_id = det
            cx_px, cy_px = int(cx_px), int(cy_px)
            # Circle at center
            cv2.circle(info_image, (cx_px, cy_px), 12, draw_color, -1)
            # Label with ID and color
            label = f"#{aruco_id} {color_name}"
            cv2.putText(info_image, label, (cx_px - 40, cy_px - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
            # Slot index
            cv2.putText(info_image, f"[{i}]", (cx_px - 10, cy_px + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.get_logger().info(
            f"ArUco colors detected: {[det[3] for det in detections[:4]]} "
            f"({len(detections)} tags found)"
        )

        return colors, info_image

    def timer_callback(self):
        if self.latest_point_cloud is None:
            return
        if self.transform_matrix_cam_to_base_link is None:
            if not self.init_transforms():
                return

        self.get_logger().info("Processing point cloud...")
        start_time = self.get_clock().now()

        # Read structured point cloud data
        structured_points = np.array(read_points(self.latest_point_cloud, field_names=("x", "y", "z"), skip_nans=True))

        if structured_points.size == 0:
            self.get_logger().warn("Received empty point cloud.")
            return
        self.get_logger().info(f"Point cloud size: {structured_points.shape[0]}")

        # ====================================TRANSFORM POINT CLOUD TO BASE LINK ====================================

        # Convert to numpy array
        points = np.vstack([structured_points['x'], structured_points['y'], structured_points['z']]).T
        points = points.astype(np.float32) # Ensure it's float32

        transformed_points_array = self.transform_points_to_base_link(points)

        end_time = self.get_clock().now()
        elapsed_time = (end_time - start_time).nanoseconds / 1e6
        self.get_logger().info(f"Point cloud processing time: {elapsed_time:.2f} ms")

        # ==================================== FILTER POINT CLOUD ====================================
        start_time = self.get_clock().now()

        filtered_point_cloud_array_in_base_link = self.filter_point_cloud_array(transformed_points_array)

        stop_time = self.get_clock().now()
        elapsed_time = (stop_time - start_time).nanoseconds / 1e6
        filtered_points_count = filtered_point_cloud_array_in_base_link.shape[0]
        self.get_logger().info(f"Filtered point cloud size: {filtered_points_count}")
        self.get_logger().info(f"Point cloud filtering time: {elapsed_time:.2f} ms \n")

        if filtered_points_count == 0:
            msg_out = NutBoxesDetection()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = 'base_link'
            msg_out.pose.position.z = -1.0  # sentinel: no detection
            msg_out.colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
            self.nutboxes_relative_position_pub.publish(msg_out)
            return

        self.publish_filtered_point_cloud(filtered_point_cloud_array_in_base_link, 'base_link')

        if filtered_points_count < 5:
            self.get_logger().warn("Not enough points to fit box rectangle.")
            return

        cx, cy, angle_rad, detected_long, detected_short = find_box_pose_2d(filtered_point_cloud_array_in_base_link)

        # Validate detected rectangle dimensions against expected box size (±20% tolerance)
        size_tolerance = 0.20
        expected_long = max(self.box_length, self.box_width)
        expected_short = min(self.box_length, self.box_width)
        long_ok  = abs(detected_long  - expected_long)  < expected_long  * size_tolerance
        short_ok = abs(detected_short - expected_short) < expected_short * size_tolerance
        if not long_ok or not short_ok:
            self.get_logger().info(
                f"❌ Rectangle size mismatch: detected ({detected_long:.3f}m x {detected_short:.3f}m), "
                f"expected ({self.box_length:.3f}m x {self.box_width:.3f}m)"
            )
            msg_out = NutBoxesDetection()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.header.frame_id = 'base_link'
            msg_out.pose.position.z = -1.0  # sentinel: no valid detection
            msg_out.colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
            self.nutboxes_relative_position_pub.publish(msg_out)
            return

        dist = float(np.hypot(cx, cy))
        self.get_logger().info(
            f"✅ Box detected: {filtered_points_count} pts | "
            f"size=({detected_long:.3f}x{detected_short:.3f})m | "
            f"pos=({cx:.3f}, {cy:.3f})m dist={dist:.3f}m angle={np.degrees(angle_rad):.1f}°"
        )

        msg_out = NutBoxesDetection()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'base_link'
        msg_out.pose.position.x = cx
        msg_out.pose.position.y = cy
        msg_out.pose.position.z = 0.0
        quat = R.from_euler('z', angle_rad).as_quat()
        msg_out.pose.orientation.x = quat[0]
        msg_out.pose.orientation.y = quat[1]
        msg_out.pose.orientation.z = quat[2]
        msg_out.pose.orientation.w = quat[3]

        # Detect nutbox colors via ArUco tags
        colors, info_image = self.detect_nutbox_colors()
        msg_out.colors = colors

        # Publish info image if available
        if info_image is not None:
            info_msg = self.cv_bridge.cv2_to_imgmsg(info_image, encoding='bgr8')
            info_msg.header.stamp = self.get_clock().now().to_msg()
            info_msg.header.frame_id = 'camera_color_optical_frame'
            self.info_image_pub.publish(info_msg)

        self.nutboxes_relative_position_pub.publish(msg_out)

    def publish_filtered_point_cloud(self, filtered_point_cloud_array, frame):
        # Create a PointCloud2 message
        filtered_point_cloud_msg = create_point_cloud2(filtered_point_cloud_array, frame)

        # Publish the filtered point cloud
        self.filtered_point_cloud_pub.publish(filtered_point_cloud_msg)
        self.get_logger().info("Filtered point cloud published")

    def init_transforms(self):
        try:
            when = rclpy.time.Time().to_msg()  # équivalent à Time(0)
            transform_cam_to_base_link = self.tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', when)
            transform_base_link_to_odom = self.tf_buffer.lookup_transform('odom','base_link', when)

            self.transform_matrix_cam_to_base_link = get_transfo_matrix(transform_cam_to_base_link)
            self.transform_matrix_base_link_to_odom = get_transfo_matrix(transform_base_link_to_odom)

            self.get_logger().info("Transforms initialized.")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transforms not initialized: {e}")
            return False

        return True

    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
        self.get_logger().info("Received point cloud data")


    def transform_points_to_base_link(self, points):
        return transform_points_array_to_frame_with_transform_matrix(points, self.transform_matrix_cam_to_base_link)
    def transform_points_to_odom(self, points):
        return transform_points_array_to_frame_with_transform_matrix(points, self.transform_matrix_base_link_to_odom)

    def filter_point_cloud_array(self, transformed_points):
        """Filter points by height, then keep only the largest dense cluster (vectorized)."""
        # Step 1: Vectorized height filter
        z = transformed_points[:, 2]
        mask = (z > self.top_plane_height_interval[0]) & (z < self.top_plane_height_interval[1])
        pts = transformed_points[mask]

        if pts.shape[0] < 5:
            return pts

        # Step 2: Voxel grid downsampling (2cm voxels) for speed
        voxel_size = 0.02
        coords_2d = pts[:, :2]
        voxel_indices = np.floor(coords_2d / voxel_size).astype(np.int32)

        # Step 3: Connected-components on voxel grid using OpenCV (ultra fast)
        # Shift to positive indices
        min_vox = voxel_indices.min(axis=0)
        voxel_indices -= min_vox
        max_vox = voxel_indices.max(axis=0)

        # Create binary occupancy image
        grid = np.zeros((max_vox[1] + 3, max_vox[0] + 3), dtype=np.uint8)
        grid[voxel_indices[:, 1] + 1, voxel_indices[:, 0] + 1] = 255

        # Dilate slightly to connect nearby voxels (bridge 1-voxel gaps)
        kernel = np.ones((3, 3), dtype=np.uint8)
        grid_dilated = cv2.dilate(grid, kernel, iterations=1)

        # Connected components
        num_labels, labels_img = cv2.connectedComponents(grid_dilated, connectivity=8)

        if num_labels <= 1:
            return pts

        # Find label of each point
        point_labels = labels_img[voxel_indices[:, 1] + 1, voxel_indices[:, 0] + 1]

        # Find largest cluster (exclude background label 0)
        unique, counts = np.unique(point_labels[point_labels > 0], return_counts=True)
        if len(unique) == 0:
            return np.empty((0, 3), dtype=pts.dtype)

        largest_label = unique[np.argmax(counts)]
        return pts[point_labels == largest_label]


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

