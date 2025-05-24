#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from std_msgs.msg import Header, Float32

from scipy.spatial.transform import Rotation as R
import numpy as np
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


def find_distance_to_platform(filtered_point_cloud_array_in_base_link):
    """
    Find the distance to the platform in the filtered point cloud.
    """
    # Calculate the distance to the platform
    distances = np.linalg.norm(filtered_point_cloud_array_in_base_link, axis=1)

    # Find the minimum distance
    min_distance = np.min(distances)

    return min_distance


class PlatformDetectionNode(Node):
    def __init__(self):
        super().__init__('platform_detection_node')

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

        # platforms
        margin = 0.02 # m
        platform_height = 0.125 # m
        self.interval_platform_height = [platform_height - margin, platform_height + margin] # m

        # distance publisher
        self.distance_pub = self.create_publisher(Float32, '/platform_distance', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Platform Detection Node started !")

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
        elapsed_time = (end_time - start_time).nanoseconds / 1e6  # Convert to milliseconds
        self.get_logger().debug(f"Point cloud processing time: {elapsed_time:.2f} ms")

        # ==================================== FILTER POINT CLOUD ====================================
        start_time = self.get_clock().now()

        filtered_point_cloud_array_in_base_link = self.filter_point_cloud_array(transformed_points_array)

        stop_time = self.get_clock().now()
        elapsed_time = (stop_time - start_time).nanoseconds / 1e6  # Convert to milliseconds
        filtered_points_count = filtered_point_cloud_array_in_base_link.shape[0]
        self.get_logger().debug(f"Filtered point cloud size: {filtered_points_count}")
        self.get_logger().debug(f"Point cloud filtering time: {elapsed_time:.2f} ms \n")

        if filtered_points_count == 0:
            msg = Float32()
            msg.data = -1.0
            self.distance_pub.publish(msg) # we always publish, even when nothing is detected so that state machine still get updated values
            return

        # transform filtered point cloud to odom frame
        filtered_point_cloud_in_odom = self.transform_points_to_odom(filtered_point_cloud_array_in_base_link)
        # for viz only :
        self.publish_filtered_point_cloud(filtered_point_cloud_in_odom, 'base_link')

        # Find the shortest from the robot to the platform
        dist_to_platform = find_distance_to_platform(filtered_point_cloud_array_in_base_link)
        self.get_logger().debug(f"Distance to platform: {dist_to_platform:.2f} m")

        msg = Float32()
        msg.data = dist_to_platform
        self.distance_pub.publish(msg)

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
            if (self.interval_platform_height[0]) < z < self.interval_platform_height[1]:
                filtered_point_cloud.append(point)

        filtered_point_cloud = np.array(filtered_point_cloud)
        return filtered_point_cloud


def main(args=None):
    rclpy.init(args=args)

    node = PlatformDetectionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()