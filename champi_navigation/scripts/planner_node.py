#!/usr/bin/env python3

from champi_navigation.path_planner import PathPlanner
from champi_navigation.world_state import WorldState
from champi_navigation.gui_node import GuiV2

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

        self.gui = GuiV2(self)
        self.gui.opponent_to_draw = self.world_state.opponent_robot
        self.gui.robot_to_draw = self.world_state.self_robot

        #debug
        self.last_path = None

    def pub_rviz_obstacles(self):
        #TODO OFFSET
        OFFSET = 0.2

        # OBSTACLE marker
        marker_obstacle = Marker()
        marker_obstacle.header.frame_id = "odom"
        marker_obstacle.type = Marker.CUBE
        marker_obstacle.action = Marker.ADD

        marker_obstacle.scale.x = float(self.world_state.opponent_robot.width+OFFSET*4)
        marker_obstacle.scale.y = float(self.world_state.opponent_robot.height+OFFSET*4)
        marker_obstacle.scale.z = 0.3

        marker_obstacle.color.a = 0.7
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


        marker_obstacle_offset.scale.x = float(self.world_state.opponent_robot.width+OFFSET*2) #TODO ou +2*OFFSET???
        marker_obstacle_offset.scale.y = float(self.world_state.opponent_robot.height+OFFSET*2)
        marker_obstacle_offset.scale.z = 0.05

        marker_obstacle_offset.color.a = 0.4
        marker_obstacle_offset.color.r = 1.0
        marker_obstacle_offset.color.g = 1.0
        marker_obstacle_offset.color.b = 0.0
        marker_obstacle_offset.pose.position.x = float(self.world_state.opponent_robot.center_x)
        marker_obstacle_offset.pose.position.y = float(self.world_state.opponent_robot.center_y)
        marker_obstacle_offset.pose.position.z = 0.0

        self.markers_pub_obstacle_offset.publish(marker_obstacle_offset)
    
    def odom_callback(self, msg):
        # print("odom callback", msg.pose.pose.position.x, msg.pose.pose.position.y)
        poseWithCov = msg.pose
        # convert to poseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = poseWithCov.pose

        self.world_state.self_robot.pose_stamped = pose_stamped

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.path_planner.set_cmd_goal(self.goal)

    def define_init_pose_callback(self, msg):
        # self.world_state.self_robot.pose_stamped.pose = msg.pose.pose
        # ic("INIT POSE RECEIVED :")
        ic(msg.pose.pose)

    def callback(self):
        self.world_state.update()
        path_msg:Path = self.path_planner.update(self.get_clock().now().to_msg())

        graph = self.path_planner.graph
        # envoi du graph pour dessiner
        # segs = []
        # if graph is not None:
        #     for key, vals in graph.items():
        #         for val in vals.keys():
        #             segs.append((key, val))
        #     for i in range(len(segs)):
        #         segs[i] = (
        #             (self.path_planner.dico_all_points[str(segs[i][0])][0], self.path_planner.dico_all_points[str(segs[i][0])][1]),
        #             (self.path_planner.dico_all_points[str(segs[i][1])][0], self.path_planner.dico_all_points[str(segs[i][1])][1])
        #         )
        #     self.gui.add_lines_to_draw(segs)

        self.path_pub.publish(path_msg)
        self.pub_rviz_obstacles()
        self.gui.callback()

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()