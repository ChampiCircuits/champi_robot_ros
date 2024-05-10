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

class Robot_Navigator():
    def __init__(self, node):
        self.navigator = BasicNavigator()
        self.node = node
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose',10)

        # self.goal_reached = False

        self.last_goal = None



    def navigate_to(self, destination: Tuple[float, float, float], max_time_allowed) -> bool:
        # log 
        self.node.get_logger().info(f"RECEVUED DEST: {destination}")
        if destination == None:
            return
        if not self.last_goal == None and self.last_goal[0] == destination[0] and self.last_goal[1] == destination[1] and self.last_goal[2] == destination[2]:
            return
        self.last_goal = destination

        # publish pose /goal_pose
        pose = PoseStamped()
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = destination[0]
        pose.pose.position.y = destination[1]
        pose.pose.orientation.z = sin(destination[2]/2)
        pose.pose.orientation.w = cos(destination[2]/2)
        self.goal_pub.publish(pose)
