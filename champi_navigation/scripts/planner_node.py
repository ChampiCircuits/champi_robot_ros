#!/usr/bin/env python3

from champi_navigation.path_planner import PathPlanner
from champi_navigation.world_state import WorldState

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from math import sin, cos


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/cmd_path', 10)

        self.timer = self.create_timer(timer_period_sec=0.02,
                                        callback=self.callback)

        # Node variables
        self.world_state = WorldState()
        self.path_planner = PathPlanner(self.world_state)


        # TODO TEMP
        self.goal = self.init_test_goal_pose()
        self.path_planner.set_cmd_goal(self.goal)

    def init_test_goal_pose(self):
        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.5
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose


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