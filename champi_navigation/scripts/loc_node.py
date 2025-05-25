#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped, Pose
import tf_transformations
from rclpy.time import Time, Duration
from tf2_ros import TransformBroadcaster
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from champi_interfaces.srv import SetPose
from math import atan2

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
            '/set_pose_rviz',
            self.set_pose_callback,
            10
        )
        self.aruco_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/aruco_loc/pose',
            self.aruco_pose_callback,
            10
        )

        # Service Server
        self.set_pose_service = self.create_service(
            SetPose,
            '/set_pose',
            self.handle_set_pose_srv,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))  
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # cooldown timer for aruco : 1s
        self.cooldown_value = 1.0 #s
        self.last_time_aruco_pose_taken_in_account = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Offset variables
        self.latest_set_pose = PoseWithCovarianceStamped()

        self.latest_robot_pose = PoseWithCovarianceStamped()
        self.robot_pose_when_set_pose = PoseWithCovarianceStamped()
        self.odom_callback(Odometry())

    def handle_set_pose_srv(self, request, response):
        self.set_pose_callback(request.pose)
        return response

    def set_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_set_pose = msg
        self.robot_pose_when_set_pose = self.latest_robot_pose
        self.get_logger().info("Set pose received")

    def aruco_pose_callback(self, msg: PoseWithCovarianceStamped):
        # We receive an aruco pose at 5Hz.
        # To avoid the pose oscillating, we update only every second
        if self.get_clock().now() - self.last_time_aruco_pose_taken_in_account < Duration(seconds=self.cooldown_value):
            return
        

        t_image = Time.from_msg(msg.header.stamp)

        # latency calculation
        t_image = Time.from_msg(msg.header.stamp)
        # t_recv  = self.get_clock().now()
        # latency_ms = (t_recv - t_image).nanoseconds / 1e6
        # print(t_recv)

        try:
            tf = self.tf_buffer.lookup_transform(
                'world',  # target frame                
                msg.header.frame_id,   
                t_image,      
                timeout=Duration(seconds=0.1)
            )
            corrected_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, tf)
            corrected_x = corrected_pose.pose.position.x
            corrected_y = corrected_pose.pose.position.y
            corrected_z = corrected_pose.pose.position.z


            self.get_logger().info(f"Corrected position (x): {corrected_x:.3f}, (y): {corrected_y:.3f}, (z): {corrected_z:.3f}")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")



        # We take the aruco pose into account only if we are almost not moving
        if abs(self.latest_robot_pose.twist.twist.linear.x) > 0.05 \
            or abs(self.latest_robot_pose.twist.twist.linear.y) > 0.05 \
                or abs(self.latest_robot_pose.twist.twist.angular.z) > 0.05:
            self.get_logger().info("Seen Aruco but robot is moving, not taking aruco pose into account")
            self.get_logger().debug(f"Robot speed: {self.latest_robot_pose.twist.twist.linear.x} m/s, {self.latest_robot_pose.twist.twist.linear.y} m/s, {self.latest_robot_pose.twist.twist.angular.z} rad/s")
            return

        # We take the aruco pose into account
        self.last_time_aruco_pose_taken_in_account = self.get_clock().now()
        self.latest_set_pose = msg
        self.robot_pose_when_set_pose = self.latest_robot_pose

        position = msg.pose.pose.position
        rotation_deg = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 180.0 / np.pi
        self.get_logger().info(f"New aruco pose received (pose={position.x} {position.y} {rotation_deg}°) (now waiting cooldown={self.cooldown_value}s)")

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