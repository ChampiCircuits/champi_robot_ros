#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from champi_navigation.kinematic_models import Robot_Kinematic_Model, Obstacle_static_model, Table_static_model
import champi_navigation.avoidance as avoidance
import champi_navigation.gui as gui

from icecream import ic
from dijkstar import Graph
from math import pi
from shapely import Point

WIDTH, HEIGHT = 900, 600  # window
TABLE_WIDTH, TABLE_HEIGHT = 300, 200  # Table size in cm
FPS = 50
OFFSET = 15 # TODO rayon du self.robot, à voir Etienne

class PoseControl(Node):

    def __init__(self):
        super().__init__('pos_control_node')



        self.robot = Robot_Kinematic_Model(TABLE_WIDTH=TABLE_WIDTH, TABLE_HEIGHT=TABLE_HEIGHT,FPS=FPS)
        self.obstacle = Obstacle_static_model(center_x=100, center_y= 100, width= 10, height= 10,offset=OFFSET)
        self.table = Table_static_model(TABLE_WIDTH, TABLE_HEIGHT, offset=OFFSET)

        self.viz = True
        self.gui = gui.Gui(self.robot, self.obstacle, self.table)


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.update()




    def check_goal_reached(self):
        error_max_lin = 1
        error_max_ang = 0.01

        if (len(self.robot.goals_positions))==0:
            return True

        self.robot.current_goal = self.robot.goals_positions[0]

        if abs(self.robot.pos[0] - self.robot.current_goal[0]) < error_max_lin and abs(self.robot.pos[1] - self.robot.current_goal[1]) < error_max_lin:
            if self.check_angle(self.robot.pos[2], self.robot.current_goal[2], error_max_ang):
                self.goto_next_goal()
                ic("GOAL REACHED")

                self.robot.goal_reached = True

    def goto(self, x, y, theta):
        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = theta - self.robot.pos[2]
        if error_theta > pi:
            error_theta -= 2*pi
        elif error_theta < -pi:
            error_theta += 2*pi
        self.robot.angular_speed = self.robot.pid_pos_theta.update(error_theta)

        # # PID
        self.robot.linear_speed = [
            self.robot.pid_pos_x.update(x - self.robot.pos[0]),
            self.robot.pid_pos_y.update(y - self.robot.pos[1])
        ]


    def goto_next_goal(self):
        # called when a goal is reached to set the new current goal
        self.robot.goal_reached = False
        self.robot.has_finished_rotation = False
        if len(self.robot.goals_positions)>0:
            self.robot.goals_positions.pop(0)
        if len(self.robot.goals_positions)>0:
            self.robot.current_goal = self.robot.goals_positions[0]
        

    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def recompute_path(self, obstacle, table, goal_pos=None):
        if goal_pos is None:
            if len(self.robot.goals_positions) > 0:
                theta = self.robot.goals_positions[-1][2] # use the theta of the goal for each point
                goal = Point(self.robot.goals_positions[-1][0],self.robot.goals_positions[-1][1])
            else:
                return
        else:
            theta = goal_pos[2]
            goal = Point(goal_pos[0],goal_pos[1])

        start = Point(self.robot.pos[0],self.robot.pos[1])
        self.robot.graph, self.robot.dico_all_points = avoidance.create_graph(start, goal, obstacle.expanded_obstacle_poly, table.expanded_poly)
        path = avoidance.find_avoidance_path(self.robot.graph, 0, 1)
        if path is not None:
            self.robot.path_nodes = path.nodes # mais en soit renvoie aussi le coût
            goals = []

            for p in self.robot.path_nodes[1:]: # we don't add the start point
                goals.append([float(self.robot.dico_all_points[p][0]),float(self.robot.dico_all_points[p][1]), theta])
            self.robot.goals_positions = goals


    def update(self):
        self.check_goal_reached()

        self.goto(self.robot.current_goal[0],
        self.robot.current_goal[1],
        self.robot.current_goal[2])
                
        self.recompute_path(self.obstacle, self.table)
        
        self.robot.update_robot_position()

        if self.viz:
            self.gui.update()


        """ENVOI MESSAGE ROS CMD_VEL"""
        """LIRE POSE GOAL, POLY ROBOT ADVERSE, ODOM"""


def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PoseControl()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()