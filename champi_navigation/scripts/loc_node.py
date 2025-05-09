#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped, Pose
import tf_transformations
from tf2_ros import TransformBroadcaster
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

def pose_to_transform(pose: Pose):
    """Convert a Pose to a tf_transformations Transform."""
    translation = (pose.position.x, pose.position.y, pose.position.z)
    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return tf_transformations.translation_matrix(translation) @ tf_transformations.quaternion_matrix(rotation)

def transform_to_pose(transform):
    """Convert a tf_transformations Transform to a Pose."""
    translation = tf_transformations.translation_from_matrix(transform)
    rotation = tf_transformations.quaternion_from_matrix(transform)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = rotation
    return pose


class LocNode(Node):
    def __init__(self):
        super().__init__('loc_node')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_otos',
            self.odom_callback,
            10
        )
        self.set_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/set_pose',
            self.set_pose_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Offset variables
        self.latest_set_pose = PoseWithCovarianceStamped()

        self.latest_robot_pose = PoseWithCovarianceStamped()
        self.robot_pose_when_set_pose = PoseWithCovarianceStamped()
        self.odom_callback(Odometry())

    def set_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_set_pose = msg
        self.robot_pose_when_set_pose = self.latest_robot_pose
        self.get_logger().info("Set pose received")

    def odom_callback(self, msg: Odometry):
        self.latest_robot_pose = msg

        t = pose_to_transform(msg.pose.pose)
        t_offset = pose_to_transform(self.latest_set_pose.pose.pose)
        t_when_set_pose = pose_to_transform(self.robot_pose_when_set_pose.pose.pose)
        t_when_set_pose_inverse = tf_transformations.inverse_matrix(t_when_set_pose)

        t_transformed = t_offset @ t_when_set_pose_inverse @ t

        # Publish odom
        new_odom = Odometry()
        new_odom.header = msg.header
        new_odom.child_frame_id = msg.child_frame_id
        new_odom.pose.pose = transform_to_pose(t_transformed)
        new_odom.pose.covariance = msg.pose.covariance
        new_odom.twist = msg.twist

        self.odom_pub.publish(new_odom)

        # Publish transform

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = new_odom.pose.pose.position.x
        transform.transform.translation.y = new_odom.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = new_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)














def main(args=None):
    rclpy.init(args=args)
    node = LocNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()