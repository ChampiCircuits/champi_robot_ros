#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odometry_topic', '/odometry')
        self.declare_parameter('output_topic', '/output')
        self.declare_parameter('yaw_velocity_threshold', 0.1)
        # Get covariance parameters (array containing the diagonal of the pose covariance matrix)
        self.declare_parameter('pose_covariance', [0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.yaw_velocity_threshold = self.get_parameter('yaw_velocity_threshold').get_parameter_value().double_value
        self.pose_covariance = self.get_parameter('pose_covariance').get_parameter_value().double_array_value

        self.cmd_vel_subscriber = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, self.odometry_topic, self.odometry_callback, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.cmd_vel = Twist()
        self.imu = Imu()
        self.odometry = Odometry()

        self.robot_stopped = False

        self.cov_36 = [0.0] * 36
        for i in range(6):
            self.cov_36[i * 6 + i] = self.pose_covariance[i]

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def imu_callback(self, msg):
        self.imu = msg

    def odometry_callback(self, msg):
        # Only update odometry if the robot is not stopped
        if not self.robot_stopped:
            self.odometry = msg

    def timer_callback(self):
        if self.is_stationary() and self.is_yaw_velocity_below_threshold():
            self.robot_stopped = True
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'odom'
            pose_msg.pose = self.odometry.pose
            pose_msg.pose.covariance = self.cov_36
            self.publisher.publish(pose_msg)
        else:
            self.robot_stopped = False

    def is_stationary(self):
        return all(value == 0 for value in [self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z, self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z])

    def is_yaw_velocity_below_threshold(self):
        return math.fabs(self.imu.angular_velocity.z) < self.yaw_velocity_threshold

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()