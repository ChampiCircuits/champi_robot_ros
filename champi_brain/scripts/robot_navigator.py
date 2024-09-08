#!/usr/bin/env python3

from rclpy.duration import Duration
from champi_interfaces.msg import ChampiPath, ChampiSegment, ChampiPoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from builtin_interfaces.msg import Time

from typing import Tuple
from math import sin, cos
from std_msgs.msg import Empty
import rclpy

from utils import pose_from_position
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.logging import get_logger

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from champi_interfaces.action import Navigate

from math import sin, cos, atan2, hypot, pi
import numpy as np

import time
from rclpy.logging import get_logger

class Robot_Navigator():
    def __init__(self, node):
        get_logger('robot_navigator').info(f"\tLaunching Robot Navigator...")
        
        self.node = node

        # Action client for /navigate
        self.action_client_navigate = ActionClient(self.node, Navigate, '/navigate')
        get_logger('robot_navigator').info('\t\tWaiting for Action Server /navigate...')
        self.action_client_navigate.wait_for_server()
        get_logger('robot_navigator').info('\t\tAction client for /navigate is ready!')

        self.gfini_pub = self.node.create_publisher(Empty, '/gfini', 10)


        self.goal_handle_navigate = None

        self.goal_pose = None
        self.last_goal = None
        get_logger('robot_navigator').info(f"\tRobot Navigator launched !")


    def navigate_to(self, destination: Tuple[float, float, float], max_time_allowed) -> bool:
        if destination is None:
            return
        if self.last_goal is not None and self.last_goal[0] == destination[0] and self.last_goal[1] == destination[1] and self.last_goal[2] == destination[2]:
            return
        self.last_goal = destination

        # publish pose /goal_pose
        pose = Pose()
        # pose.header.stamp.sec = 0
        # pose.header.stamp.nanosec = 0
        pose.position.x = destination[0]
        pose.position.y = destination[1]
        pose.orientation.z = sin(destination[2]/2)
        pose.orientation.w = cos(destination[2]/2)

        goal_pose = ChampiPoint()
        goal_pose.name = ""
        goal_pose.pose = pose
        goal_pose.point_type = 1 # TODO use enum
        goal_pose.tolerance = 0.5 # TODO use enum
        goal_pose.robot_should_stop_here = True

        segment = ChampiSegment()
        segment.name = ""
        segment.start = goal_pose
        segment.end = goal_pose
        segment.speed = 0.3
        segment.look_at_point = goal_pose

        path = Navigate.Goal()
        path.path.name = ""
        path.path.segments = [segment]
        path.path.max_time_allowed = max_time_allowed


        print(path)
        # Send the goal to the action server
        future_navigate_result = self.action_client_navigate.send_goal_async(path, feedback_callback=self.feedback_callback)

        # Add done callback
        future_navigate_result.add_done_callback(self.goal_response_callback)

        get_logger('robot_navigator').info('Goal sent from robot_navigator!')

    # ==================================== ROS2 Callbacks ==========================================

    def feedback_callback(self, feedback_msg):
        get_logger('robot_navigator').info(f'Feedback received! Feedback: {feedback_msg}')


    # ==================================== Done Callbacks ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            get_logger('robot_navigator').info('Goal rejected :(')
            return
        
        self.goal_handle_navigate = goal_handle

        get_logger('robot_navigator').info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        get_logger('robot_navigator').error(f'Received a result !')
        result = future.result().result
        get_logger('robot_navigator').warn(f'Result: {result.success}, {result.message}')
        if result.success:
            msg = Empty()
            self.gfini_pub.publish(msg)
        # else:
        #     print(result.)

    def cancel_done_callback(self, future):
        get_logger('robot_navigator').info('Goal cancelled succesfully!')
        get_logger('robot_navigator').error('TODO FINIR ICI')
        # self.send_goal(self.goal_pose)


# ============================================ Utils ==============================================


    def send_goal(self, goal_pose):
        # Create a Navigate request
        goal = Navigate.Goal()
        goal.poses = [goal_pose]

        # Send the goal to the action server
        future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)

        # Add done callback
        future_navigate_result.add_done_callback(self.goal_response_callback)

        get_logger('robot_navigator').info('Goal sent!')

