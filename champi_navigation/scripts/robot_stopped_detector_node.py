#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from rclpy.executors import ExternalShutdownException
from math import acos


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

        # self.cmd_vel_subscriber = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, self.odometry_topic, self.odometry_callback, 10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)

        self.timer = self.create_timer(0.2, self.timer_callback)  # 10Hz

        self.cmd_vel = Twist()
        self.imu = Imu()
        self.odometry = Odometry()

        self.robot_stopped = False

        self.cov_36 = [0.0] * 36
        for i in range(6):
            self.cov_36[i * 6 + i] = self.pose_covariance[i]
        
        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = 'odom'
        self.pose_msg.pose.covariance = self.cov_36

        self.req_imu = True
        self.req_odom = True
        self.req_pose = True

        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.update_robot_pose, 10)
        self.last_robot_pose = None
        self.robot_pose = None
        self.has_been_set_pose = False

    def update_robot_pose(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, acos(msg.pose.pose.orientation.w)*2*180/3.1415)
        if self.diff(self.robot_pose, self.last_robot_pose) > 0.10:
            self.has_been_set_pose = True
            self.last_robot_pose = self.robot_pose
        # get_logger('rclpy').info(f"updated robot_pose: {self.robot_pose}")
        
    def diff(self, pose, last_pose):
        return math.sqrt(math.pow(pose[0]-last_pose[0],2) + math.pow(pose[1]-last_pose[1],2))
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def imu_callback(self, msg):
        self.imu = msg

    def odometry_callback(self, msg):
        # Only update odometry if the robot is not stopped
        if not self.robot_stopped:
            self.odometry = msg

    def timer_callback(self):
        # if self.is_stationary() and self.is_yaw_velocity_below_threshold():

        # si on a recu un appel au service set pose dans la derniere seconde, alors on republish pas la pose
        # en fait nan, si la difference est supérieure à 10cm, c'est qu'on doit avoir recu un set pose
        if self.has_been_set_pose:
            self.has_been_set_pose = False
            return

        if self.is_yaw_velocity_below_threshold():
            self.robot_stopped = True
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.pose = self.odometry.pose
            self.publisher.publish(self.pose_msg)
        else:
            self.robot_stopped = False

    def is_stationary(self):
        return self.cmd_vel.linear.x == 0 and self.cmd_vel.linear.y == 0 and self.cmd_vel.angular.z == 0

    def is_yaw_velocity_below_threshold(self):
        return math.fabs(self.imu.angular_velocity.z) < self.yaw_velocity_threshold

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()