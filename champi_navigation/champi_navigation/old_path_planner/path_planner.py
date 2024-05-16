#!/usr/bin/env python3

from shapely import Point
from icecream import ic
from math import sin, cos, atan2

from champi_navigation import avoidance
import champi_navigation.trajectory as trajectory

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose

TABLE_WIDTH, TABLE_HEIGHT = 3, 2  # Table size in m

class PathPlanner:
    def __init__(self, world_state):

        self.trajectory_builder = trajectory.TrajectoryBuilder(world_state)
        self.world_state = world_state

        self.cmd_goal = None
        self.enable_avoidance = False

        # self.environment_state = None
        # self.robot_state = None

        # debug
        self.graph = None
        # self.dico_all_points = {}
        # self.path_nodes = None

    

    def set_cmd_goal(self, goal):
        """Set the goal of the robot (x, y, theta)
            self.update() should be called to compute a new path"""
        self.cmd_goal = goal
        ic("pp: RECEIVED NEW CMD GOAL :")
        ic(self.cmd_goal)

    def update(self, current_time) -> Path:
        """Spin once of the planning loop"""
        if self.cmd_goal is None:
            return Path()
        
        if self.enable_avoidance:
            cmd_path = self.compute_path_avoidance()
        else:
            cmd_path = self.compute_path_simple()
        
        # ic("PATH COMPUTED :")
        # ic(cmd_path)

        # convert the cmd_path [[x, y, theta],...] to a Path ros msg
        cmd_path_msg = Path()
        cmd_path_msg.header.stamp = current_time
        cmd_path_msg.header.frame_id = "odom"
        for pose in cmd_path:
            p = PoseStamped()
            p.pose.position.x = pose[0]
            p.pose.position.y = pose[1]
            p.pose.position.z = 0.
            p.pose.orientation.x = 0.
            p.pose.orientation.y = 0.
            p.pose.orientation.z = sin(pose[2]/2)
            p.pose.orientation.w = cos(pose[2]/2)
            cmd_path_msg.poses.append(p)


        # ic("PATH MSG :")
        # ic(cmd_path_msg)

        return cmd_path_msg


    def compute_path_simple(self):
        """Return a direct path from the current robot position to the goal.
        Must be called with a goal != None."""
        p_stamped = self.world_state.self_robot.pose_stamped
        goal = self.cmd_goal
        cmd_path = [(p_stamped.pose.position.x,p_stamped.pose.position.y,2 * atan2(p_stamped.pose.orientation.z, p_stamped.pose.orientation.w)), 
                    (goal.position.x, goal.position.y, 2 * atan2(goal.orientation.z, goal.orientation.w))]
        return cmd_path


    def compute_path_avoidance(self):
        """Must be called with a goal != None."""

        # ic("COMPUTE PATH AVOIDANCE")

        goal = Point(self.cmd_goal.position.x, self.cmd_goal.position.y)
        theta = self.cmd_goal.orientation.z # TODO sûr ?
        start = Point(self.world_state.self_robot.pose_stamped.pose.position.x,
                      self.world_state.self_robot.pose_stamped.pose.position.y)

        # ic(start, goal, theta)

        self.graph, self.dico_all_points = avoidance.create_graph(start, goal, self.world_state)
        # print("GRAPH CREATED")
        # print(self.graph)
        path = avoidance.find_avoidance_path_(self.graph, "0", "1")
        # ic("PATH FOUND")
        
        if path is not None:
            self.path_nodes = path.nodes # note : return also the costs
            
            goals = []
            for p in self.path_nodes:
                goals.append([float(self.dico_all_points[str(p)][0]),float(self.dico_all_points[str(p)][1]), theta])
            # ic("path found")
            return goals
        else:
            # print("\n")
            # print(15*"#")
            # print("No path found de",start ,"à", goal)
            # print(self.obstacle.expanded_obstacle_poly)
            # print("graph :")
            # for key, val in self.graph.items():
            #     print(key,"--", val.keys())
            # print("dico points :")
            # print(self.dico_all_points)
            # print("\n\n")

            # #create an img of the table and the obstacle
            # import matplotlib.pyplot as plt
            # fig, ax = plt.subplots()
            # ax.plot(*self.obstacle.expanded_obstacle_poly.exterior.xy)
            # ax.plot(*self.table.expanded_poly.exterior.xy)
            # ax.plot(start.x, start.y, 'o')
            # ax.plot(goal.x, goal.y, 'o')
            # plt.show()

            return []