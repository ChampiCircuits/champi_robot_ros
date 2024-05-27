#!/usr/bin/env python3

from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from builtin_interfaces.msg import Time

from typing import Tuple
from math import sin, cos
from std_msgs.msg import Empty
import rclpy
from icecream import ic

from utils import pose_from_position
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from champi_interfaces.action import Navigate

from math import sin, cos, atan2, hypot, pi
import numpy as np

import time
from icecream import ic

class Robot_Navigator():
    def __init__(self, node):
        self.node = node

        # Action client for /navigate
        self.action_client_navigate = ActionClient(self.node, Navigate, '/navigate')
        self.action_client_navigate.wait_for_server()
        self.node.get_logger().info('Action client for /navigate is ready!')

        self.goal_handle_navigate = None

        self.goal_pose = None
        self.last_goal = None


    def navigate_to(self, destination: Tuple[float, float, float], max_time_allowed) -> bool:
        if destination is None:
            return
        if not self.last_goal is None and self.last_goal[0] == destination[0] and self.last_goal[1] == destination[1] and self.last_goal[2] == destination[2]:
            return
        self.last_goal = destination


    # ==================================== ROS2 Callbacks ==========================================

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f'Feedback received! Feedback: {feedback_msg}')


    # ==================================== Done Callbacks ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return
        
        self.goal_handle_navigate = goal_handle

        self.node.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Result: {result.success}, {result.message}')


    def cancel_done_callback(self, future):
        self.node.get_logger().info('Goal cancelled succesfully!')
        self.send_goal(self.goal_pose)


# ============================================ Utils ==============================================


    def send_goal(self, goal_pose):
        # Create a Navigate request
        goal = Navigate.Goal()
        goal.poses = [goal_pose]

        # Send the goal to the action server
        future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)

        # Add done callback
        future_navigate_result.add_done_callback(self.goal_response_callback)

        self.node.get_logger().info('Goal sent!')

