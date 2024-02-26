#!/usr/bin/env python3

from math import pi

from shapely import Point
from icecream import ic

import champi_navigation.avoidance as avoidance
from champi_navigation.pid import PID
from champi_navigation.kinematic_models import Obstacle_static_model, Table_static_model
from champi_navigation.utils import Vel, RobotState


TABLE_WIDTH, TABLE_HEIGHT = 3, 2  # Table size in m
OFFSET = 0.15 # TODO rayon du self.robot, à voir Etienne


class CmdVelCalculator:
    def __init__(self):
        # PIDs
        self.pid_pos_x = PID(1, 0, 0)
        self.pid_pos_y = PID(1, 0, 0)
        self.pid_pos_theta = PID(1, 0, 0,)
    

    def compute_cmd_vel(self, robot_state, pose_goal, dt):
        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = pose_goal[2] - robot_state.current_pose[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2*pi
            else:
                error_theta += 2*pi
        
        cmd_vel = Vel()

        cmd_vel.theta = self.pid_pos_theta.update(error_theta, dt)

        # # PID
        cmd_vel.x = self.pid_pos_x.update(pose_goal[0] - robot_state.current_pose[0], dt)
        cmd_vel.y = self.pid_pos_y.update(pose_goal[1] - robot_state.current_pose[1], dt)

        return cmd_vel

class CmdVelCalculator2:
    def __init__(self):
        # PIDs
        self.pid_angle_vec_dir = PID(1, 0, 0)
    

    def compute_cmd_vel(self, robot_state, pose_goal, dt):
        # if it's shorter to turn in the other direction, we do it
        # TODO not always working
        error_theta = pose_goal[2] - robot_state.current_pose[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2*pi
            else:
                error_theta += 2*pi
        
        cmd_vel = Vel()

        

        return cmd_vel
        
class PoseControl:
    def __init__(self, viz=None):

        self.viz = viz
        
        # Objects instanciation
        
        self.obstacle = Obstacle_static_model(center_x=1, center_y= 1, width= 0.1, height= 0.1,offset=OFFSET)
        self.table = Table_static_model(TABLE_WIDTH, TABLE_HEIGHT, offset=OFFSET)
        
        self.robot_state = RobotState()
        self.cmd_vel_calc = CmdVelCalculator()


        # Variables related to goals
        
        self.goal_reached = True
        self.has_finished_rotation = False

        self.current_goal = self.robot_state.current_pose.tolist() # TODO
        
        self.cmd_path = []  # [[x, y, theta], ...]
        self.cmd_path.append(self.current_goal)

        # Variables related to avoidance
        self.graph = None
        self.dico_all_points = {}
        self.path_nodes = None


    def set_goal(self, goal):
        self.current_goal = goal
        # Clear existing goals
        self.cmd_path = [goal]


    def control_loop_spin_once(self, dt):

        self.update_goal()
        # self.recompute_path(self.obstacle, self.table)

        cmd_vel = self.cmd_vel_calc.compute_cmd_vel(self.robot_state, self.current_goal, dt)

        # Viz
        if self.viz is not None:
            self.viz.draw_goal_poses(self.robot_state.current_pose, self.cmd_path)
            self.viz.update()
        
        # publish the velocity (expressed in the base_link frame)
        cmd_vel = Vel.to_robot_frame(self.robot_state.current_pose, cmd_vel)    
        return cmd_vel.to_twist()


    def update_goal(self):
        """Checks if the goal is reached and switch to the next one if it is the case."""

        error_max_lin = 1
        error_max_ang = 0.01

        if (len(self.cmd_path))==0:
            return

        self.current_goal = self.cmd_path[0]

        if abs(self.robot_state.current_pose[0] - self.current_goal[0]) < error_max_lin and abs(self.robot_state.current_pose[1] - self.current_goal[1]) < error_max_lin:
            if self.check_angle(self.robot_state.current_pose[2], self.current_goal[2], error_max_ang):
                self.switch_to_next_goal()
                ic("GOAL REACHED")

                self.goal_reached = True


    def switch_to_next_goal(self):
        # called when a goal is reached to set the new current goal
        if len(self.cmd_path)>0:
            self.cmd_path.pop(0)
        if len(self.cmd_path)>0:

            self.current_goal = self.cmd_path[0]

            self.goal_reached = False
            self.has_finished_rotation = False # TODO il n'est jamais remis à true !
        

    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def recompute_path(self, obstacle, table, goal_pos=None):
        if goal_pos is None:
            if len(self.cmd_path) > 0:
                theta = self.cmd_path[-1][2] # use the theta of the goal for each point
                goal = Point(self.cmd_path[-1][0],self.cmd_path[-1][1])
            else:
                return
        else:
            theta = goal_pos[2]
            goal = Point(goal_pos[0],goal_pos[1])

        start = Point(self.robot_state.current_pose[0],self.robot_state.current_pose[1])
        self.graph, self.dico_all_points = avoidance.create_graph(start, goal, obstacle.expanded_obstacle_poly, table.expanded_poly)
        path = avoidance.find_avoidance_path(self.graph, 0, 1)
        if path is not None:
            self.path_nodes = path.nodes # mais en soit renvoie aussi le coût
            goals = []

            for p in self.path_nodes[1:]: # we don't add the start point
                goals.append([float(self.dico_all_points[p][0]),float(self.dico_all_points[p][1]), theta])
            self.cmd_path = goals
    