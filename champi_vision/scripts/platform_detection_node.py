#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import PoseStamped

from champi_interfaces.msg import NutBoxesDetection

from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
from icecream import ic

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

    def timer_callback(self):
        if self.latest_point_cloud is None:
            return
        if self.transform_matrix_cam_to_base_link is None:
            if not self.init_transforms():
                return

        self.get_logger().debug("Processing point cloud...")
        start_time = self.get_clock().now()

        # Read structured point cloud data
        structured_points = np.array(read_points(self.latest_point_cloud, field_names=("x", "y", "z"), skip_nans=True))

        if structured_points.size == 0:
            self.get_logger().warn("Received empty point cloud.")
            return
        self.get_logger().debug(f"Point cloud size: {structured_points.shape[0]}")

        # ====================================TRANSFORM POINT CLOUD TO BASE LINK ====================================

        # Convert to numpy array
        points = np.vstack([structured_points['x'], structured_points['y'], structured_points['z']]).T
        points = points.astype(np.float32) # Ensure it's float32

        transformed_points_array = self.transform_points_to_base_link(points)

        end_time = self.get_clock().now()
        elapsed_time = (end_time - start_time).nanoseconds / 1e6
        self.get_logger().debug(f"Point cloud processing time: {elapsed_time:.2f} ms")

        # ==================================== FILTER POINT CLOUD ====================================
        start_time = self.get_clock().now()

        filtered_point_cloud_array_in_base_link = self.filter_point_cloud_array(transformed_points_array)

        stop_time = self.get_clock().now()
        elapsed_time = (stop_time - start_time).nanoseconds / 1e6
        filtered_points_count = filtered_point_cloud_array_in_base_link.shape[0]
        self.get_logger().debug(f"Filtered point cloud size: {filtered_points_count}")
        self.get_logger().debug(f"Point cloud filtering time: {elapsed_time:.2f} ms \n")

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
        long_ok  = abs(detected_long  - self.box_length) < self.box_length * size_tolerance
        short_ok = abs(detected_short - self.box_width)  < self.box_width  * size_tolerance
        if not long_ok or not short_ok:
            self.get_logger().debug(
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
        self.get_logger().debug(
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
        # TODO: implement actual color detection for each nutbox
        msg_out.colors = [NutBoxesDetection.COLOR_UNKNOWN] * 4
        self.nutboxes_relative_position_pub.publish(msg_out)

    def publish_filtered_point_cloud(self, filtered_point_cloud_array, frame):
        # Create a PointCloud2 message
        filtered_point_cloud_msg = create_point_cloud2(filtered_point_cloud_array, frame)

        # Publish the filtered point cloud
        self.filtered_point_cloud_pub.publish(filtered_point_cloud_msg)
        self.get_logger().debug("Filtered point cloud published")

    def init_transforms(self):
        try:
            when = rclpy.time.Time().to_msg()  # équivalent à Time(0)
            transform_cam_to_base_link = self.tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', when)
            transform_base_link_to_odom = self.tf_buffer.lookup_transform('odom','base_link', when)

            self.transform_matrix_cam_to_base_link = get_transfo_matrix(transform_cam_to_base_link)
            self.transform_matrix_base_link_to_odom = get_transfo_matrix(transform_base_link_to_odom)

            self.get_logger().info("Transforms initialized.")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Transforms not initialized.")
            return False

        return True

    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
        self.get_logger().debug("Received point cloud data")


    def transform_points_to_base_link(self, points):
        return transform_points_array_to_frame_with_transform_matrix(points, self.transform_matrix_cam_to_base_link)
    def transform_points_to_odom(self, points):
        return transform_points_array_to_frame_with_transform_matrix(points, self.transform_matrix_base_link_to_odom)

    def filter_point_cloud_array(self, transformed_points):
        filtered_point_cloud = []
        for point in transformed_points:
            x, y, z = point
            if self.top_plane_height_interval[0] < z < self.top_plane_height_interval[1]:
                filtered_point_cloud.append(point)

        filtered_point_cloud = np.array(filtered_point_cloud)
        return filtered_point_cloud


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