#!/usr/bin/env python3

from shapely import Point
from icecream import ic

import champi_navigation.avoidance as avoidance
from champi_navigation.kinematic_models import Obstacle_static_model, Table_static_model


TABLE_WIDTH, TABLE_HEIGHT = 3, 2  # Table size in m
OFFSET = 0.15 # TODO rayon du self.robot, à voir Etienne

class PathPlanner:
    def __init__(self, enable_avoidance):

        self.enable_avoidance = enable_avoidance
        
        self.obstacle = Obstacle_static_model(center_x=1.5, center_y= 1, width= 0.1, height= 0.1, offset=OFFSET)
        self.table = Table_static_model(TABLE_WIDTH, TABLE_HEIGHT, offset=-OFFSET)

        self.robot_state = None

        self.graph = None
        self.dico_all_points = {}
        self.path_nodes = None

        self.cmd_goal = None
    

    def set_cmd_goal(self, goal):
        """Set the goal of the robot (x, y, theta)"""
        self.cmd_goal = goal
    
    def set_robot_state(self, robot_state):
        self.robot_state = robot_state
    

    def planning_loop_spin_once(self):
        """Spin once of the planning loop"""

        if self.cmd_goal is None:
            return []
        
        if self.enable_avoidance:
            cmd_path = self.compute_path_avoidance()
        else:
            cmd_path = self.compute_path_simple()
        
        return cmd_path


    def compute_path_simple(self):
        """Compute a simple path from the current robot position to the goal.
        Must be called with a goal != None."""
        cmd_path = [self.robot_state.current_pose, self.cmd_goal]
        return cmd_path


    def compute_path_avoidance(self):
        """Must be called with a goal != None."""

        goal = Point(self.cmd_goal[0], self.cmd_goal[1])
        theta = self.cmd_goal[2]
        start = Point(self.robot_state.current_pose[0],self.robot_state.current_pose[1])

        self.graph, self.dico_all_points = avoidance.create_graph(start, goal, self.obstacle.expanded_obstacle_poly, self.table.expanded_poly)
        path = avoidance.find_avoidance_path(self.graph, 0, 1)
        
        if path is not None:
            self.path_nodes = path.nodes # mais en soit renvoie aussi le coût
            
            goals = []
            for p in self.path_nodes:
                goals.append([float(self.dico_all_points[p][0]),float(self.dico_all_points[p][1]), theta])
            self.cmd_path = goals
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