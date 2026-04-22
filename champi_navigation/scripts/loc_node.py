#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
from rclpy.duration import Duration
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Pose, Vector3Stamped
from std_msgs.msg import Float32
from champi_interfaces.srv import SetPose
import tf2_ros
import tf_transformations
from tf2_ros import TransformBroadcaster
from math import atan2, degrees, sqrt, pi


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
            self.odom_otos_callback,
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
        self.aruco_cooldown_s = 1.0
        self.last_aruco_correction_time = self.get_clock().now()

        # Buffer for past OTOS poses (latency compensation, ~5s at 100 Hz)
        self.odom_otos_buffer: deque = deque(maxlen=500)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # Innovation = différence entre pose estimée courante et pose ArUco reçue
        # (vector.x = dx, vector.y = dy en mètres, vector.z = dtheta en radians)
        self.innovation_pub = self.create_publisher(Vector3Stamped, '/loc/aruco_innovation', 10)
        self.latency_pub = self.create_publisher(Float32, '/loc/aruco_latency_ms', 10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Offset variables
        self.odom_aruco = PoseWithCovarianceStamped()

        self.latest_robot_pose = Odometry()
        self.odom_otos_at_aruco_capture = Odometry()
        self.odom_otos_callback(Odometry())

    def handle_set_pose_srv(self, request, response):
        self.set_pose_callback(request.pose)
        return response

    def set_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.odom_aruco = msg
        self.odom_otos_at_aruco_capture = self.latest_robot_pose
        self.get_logger().warn("📍 Set pose received")

    def aruco_pose_callback(self, msg: PoseWithCovarianceStamped):
        # We receive an aruco pose at 5Hz.
        # To avoid the pose oscillating, we update only every second
        if self.get_clock().now() - self.last_aruco_correction_time < Duration(seconds=self.aruco_cooldown_s):
            return

        # Take the aruco pose into account using the OTOS pose at image capture time (latency compensation)
        self.last_aruco_correction_time = self.get_clock().now()

        # --- Innovation: écart entre pose estimée courante et pose ArUco reçue ---
        t_cur_world = (pose_to_transform(self.odom_aruco.pose.pose)
                       @ tf_transformations.inverse_matrix(pose_to_transform(self.odom_otos_at_aruco_capture.pose.pose))
                       @ pose_to_transform(self.latest_robot_pose.pose.pose))
        cur_pose = transform_to_pose(t_cur_world)
        dx = msg.pose.pose.position.x - cur_pose.position.x
        dy = msg.pose.pose.position.y - cur_pose.position.y
        new_theta = 2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        cur_theta = 2.0 * atan2(cur_pose.orientation.z, cur_pose.orientation.w)
        dtheta = (new_theta - cur_theta + pi) % (2 * pi) - pi
        dist = sqrt(dx**2 + dy**2)

        if dist > 0.05:
            self.get_logger().warn(
                f"🚨⚠️ ArUco correction jump: {dist*100:.1f}cm "
                f"(dx={dx:.3f}m dy={dy:.3f}m dθ={degrees(dtheta):.1f}°)"
            )
        self.get_logger().info(
            f"ArUco innovation: dx={dx:.3f}m dy={dy:.3f}m dθ={degrees(dtheta):.1f}°"
        )
        innov_msg = Vector3Stamped()
        innov_msg.header.stamp = msg.header.stamp
        innov_msg.header.frame_id = 'odom'
        innov_msg.vector.x = dx
        innov_msg.vector.y = dy
        innov_msg.vector.z = dtheta
        self.innovation_pub.publish(innov_msg)
        # --- fin innovation ---

        self.odom_aruco = msg
        self.odom_otos_at_aruco_capture = self._get_odom_at_stamp(msg.header.stamp)

        position = msg.pose.pose.position
        rotation_deg = degrees(atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.get_logger().info(f"New aruco pose received (pose={position.x} {position.y} {rotation_deg}°) (now waiting cooldown={self.aruco_cooldown_s}s)")

    def _get_odom_at_stamp(self, stamp_msg) -> Odometry:
        """Return the buffered Odometry sample closest to stamp_msg for latency compensation."""
        target_ns = Time.from_msg(stamp_msg).nanoseconds
        if not self.odom_otos_buffer:
            return self.latest_robot_pose
        closest_ts, closest_odom = min(self.odom_otos_buffer, key=lambda e: abs(e[0] - target_ns))
        age_ms = abs(closest_ts - target_ns) / 1e6
        if age_ms > 500.0:
            self.get_logger().warn(
                f"ArUco stamp is {age_ms:.0f}ms away from closest OTOS sample — using current pose instead"
            )
            return self.latest_robot_pose
        self.latency_pub.publish(Float32(data=float(age_ms)))
        return closest_odom

    def odom_otos_callback(self, msg: Odometry):
        self.latest_robot_pose = msg
        self.odom_otos_buffer.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))

        t_odom_aruco_world = pose_to_transform(self.odom_aruco.pose.pose)
        
        t_odom_otos_now = pose_to_transform(msg.pose.pose)
        t_odom_otos_at_capture = pose_to_transform(self.odom_otos_at_aruco_capture.pose.pose)
        t_odom_otos_at_capture_inv = tf_transformations.inverse_matrix(t_odom_otos_at_capture)

        t_robot_world = t_odom_aruco_world @ t_odom_otos_at_capture_inv @ t_odom_otos_now

        # Publish odom
        new_odom = Odometry()
        new_odom.header = msg.header
        new_odom.child_frame_id = msg.child_frame_id
        new_odom.pose.pose = transform_to_pose(t_robot_world)
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