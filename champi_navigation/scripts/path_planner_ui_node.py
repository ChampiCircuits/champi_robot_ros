#!/usr/bin/env python3

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



class PathPlannerUINode(Node):

    def __init__(self):
        super().__init__('planner_ui')

        # Subscriber for /goal_pose
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Action client for /navigate
        self.action_client_navigate = ActionClient(self, Navigate, '/navigate')
        self.action_client_navigate.wait_for_server()
        self.get_logger().info('Action client for /navigate is ready!')

        self.future_navigate_result = None
        self.get_result_future = None
    

    # ==================================== ROS2 Callbacks ==========================================

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'New goal received!')

        # TODO cancel current if self.future_navigate_result not None
        
        # Create a Navigate request
        goal = Navigate.Goal()
        goal.poses = [msg.pose]

        # Send the goal to the action server
        self.future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)

        # Add done callback
        self.future_navigate_result.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Goal sent!')


    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received! Feedback: {feedback_msg}')


    # ==================================== Done Callback ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')






 


def main(args=None):
    rclpy.init(args=args)
    planner_ui = PathPlannerUINode()
    rclpy.spin(planner_ui)

    planner_ui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
