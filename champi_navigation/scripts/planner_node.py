#!/usr/bin/env python3

from champi_navigation.path_planner import PathPlanner
from champi_navigation.world_state import WorldState

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from math import sin, cos
from icecream import ic

class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/cmd_path', 10)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.define_init_pose_callback, 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(timer_period_sec=0.02,
                                        callback=self.callback)

        # Node variables
        self.world_state = WorldState()
        self.path_planner = PathPlanner(self.world_state)

    
    def odom_callback(self, msg):
        self.world_state.self_robot.pose_stamped.pose = msg.pose

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.path_planner.set_cmd_goal(self.goal)

    def define_init_pose_callback(self, msg):
        self.world_state.self_robot.pose_stamped.pose = msg.pose.pose
        ic("INIT POSE RECEIVED :")
        ic(msg.pose.pose)

    def callback(self):
        self.world_state.update()
        path_msg:Path = self.path_planner.update(self.get_clock().now().to_msg())
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()