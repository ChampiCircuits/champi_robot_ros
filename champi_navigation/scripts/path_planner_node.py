#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String

from math import sin, cos, atan2, hypot, pi
import numpy as np

import time
from icecream import ic

from champi_navigation.path_planner import AStarPathPlanner, ComputePathResult


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        # String publisher for feedback TODO implement Actions later
        self.feedback_pub = self.create_publisher(String, '/planner_feedback', 10)

        loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value


        self.timer = self.create_timer(timer_period_sec=loop_period,
                                       callback=self.timer_callback)
        self.robot_pose = None
        self.goal_pose = None
        self.costmap = None

        self.path_planner = None

    # ==================================== ROS2 Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        ]

    def goal_callback(self, msg):
        self.goal_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            2 * atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        ]

    def costmap_callback(self, msg):

        if self.path_planner is None:
            self.path_planner = AStarPathPlanner(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    # ==================================== Main Loop ==========================================

    def timer_callback(self):

        # We're still waiting for first messages (init)
        if self.robot_pose is None or self.costmap is None:
            self.feedback_pub.publish(String(data='INIT'))
            return

        if self.goal_pose is None:
            self.feedback_pub.publish(String(data='FREE'))
            return

        # Check if the goal is reached
        if self.is_current_goal_reached():
            self.goal_pose = None
            return

        # Compute the path
        path_msg, result = self.path_planner.compute_path(self.robot_pose, self.goal_pose, self.costmap)

        if result != ComputePathResult.SUCCESS:
            self.get_logger().warn(f'Path computation failed: {result.name}')

        # Publish the path and feedback
        self.path_pub.publish(path_msg)
        self.feedback_pub.publish(String(data=result.name))

    # ====================================== Utils ==========================================

    def is_current_goal_reached(self):
        """
        Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow.
        """

        error_max_lin = 0.05
        error_max_ang = 0.05

        return (abs(self.robot_pose[0] - self.goal_pose[0]) < error_max_lin
                and abs(self.robot_pose[1] - self.goal_pose[1]) < error_max_lin
                and self.check_angle(self.robot_pose[2], self.goal_pose[2], error_max_ang))

    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if abs(2 * pi - error) < 0.01:
            error = 0
        return error < error_max


def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
