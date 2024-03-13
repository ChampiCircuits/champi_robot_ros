#!/usr/bin/env python3

from champi_navigation.path_planner import PathPlanner
from champi_navigation.world_state import WorldState

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from math import sin, cos
from icecream import ic

class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/cmd_path', 10)
        self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.define_init_pose_callback, 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # viz only
        self.markers_pub_obstacle = self.create_publisher(Marker, "/visualization_marker_obstacle", 10)
        self.markers_pub_obstacle_offset = self.create_publisher(Marker, "/visualization_marker_obstacle_offset", 10)

        self.timer = self.create_timer(timer_period_sec=0.02,
                                        callback=self.callback)

        # Node variables
        self.world_state = WorldState()
        self.path_planner = PathPlanner(self.world_state)



    def pub_rviz_obstacles(self):
        # OBSTACLE marker
        marker_obstacle = Marker()
        marker_obstacle.header.frame_id = "odom"
        marker_obstacle.type = Marker.CUBE
        marker_obstacle.action = Marker.ADD

        marker_obstacle.scale.x = float(self.world_state.opponent_robot.width)
        marker_obstacle.scale.y = float(self.world_state.opponent_robot.height)
        marker_obstacle.scale.z = 0.3

        marker_obstacle.color.a = 1.0
        marker_obstacle.color.r = 0.5
        marker_obstacle.color.g = 0.0
        marker_obstacle.color.b = 0.0
        marker_obstacle.pose.position.x = float(self.world_state.opponent_robot.center_x)
        marker_obstacle.pose.position.y = float(self.world_state.opponent_robot.center_y)
        marker_obstacle.pose.position.z = 0.0

        self.markers_pub_obstacle.publish(marker_obstacle)


        # OFFSET OBSTACLE MARKER
        marker_obstacle_offset = Marker()
        marker_obstacle_offset.header.frame_id = "odom"
        marker_obstacle_offset.type = Marker.CUBE
        marker_obstacle_offset.action = Marker.ADD

        #TODO OFFSET
        OFFSET = 0.1
        marker_obstacle_offset.scale.x = float(self.world_state.opponent_robot.width+2*OFFSET)
        marker_obstacle_offset.scale.y = float(self.world_state.opponent_robot.height+2*OFFSET)
        marker_obstacle_offset.scale.z = 0.05

        marker_obstacle_offset.color.a = 1.0
        marker_obstacle_offset.color.r = 1.0
        marker_obstacle_offset.color.g = 1.0
        marker_obstacle_offset.color.b = 0.0
        marker_obstacle_offset.pose.position.x = float(self.world_state.opponent_robot.center_x)
        marker_obstacle_offset.pose.position.y = float(self.world_state.opponent_robot.center_y)
        marker_obstacle_offset.pose.position.z = 0.0

        self.markers_pub_obstacle_offset.publish(marker_obstacle_offset)
    
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
        self.pub_rviz_obstacles()


def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()