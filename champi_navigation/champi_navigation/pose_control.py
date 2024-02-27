#!/usr/bin/env python3

from math import pi, atan2, sqrt
from icecream import ic
from enum import Enum
import time

from champi_navigation.utils import Vel, RobotState
from champi_navigation.pid import PID

import matplotlib.pyplot as plt


class CmdVelUpdater:
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

class CmdVelUpdater2:
    def __init__(self):
        # PIDs
        # self.pid_angle_vec_dir = PID(1, 0, 0)
        self.start_pose = None
        self.goal_pose = None
        
        self.vel_profile_mag = TrapezoidalVelocityProfile(0.5, 0.5)
        self.vel_profile_theta = TrapezoidalVelocityProfile(2.0, 1.0)
    
    def compute_cmd_vel(self, robot_state, pose_goal, dt):

        # Quick fix parce qu'il faut avoir la pose de depart à vitesse = 0
        if self.goal_pose != pose_goal:

            self.start_pose = robot_state.current_pose
            self.goal_pose = pose_goal

            distance_start_to_goal = ((pose_goal[0] - self.start_pose[0])**2 + (pose_goal[1] - self.start_pose[1])**2)**0.5
            self.vel_profile_mag.set_new_goal(distance_start_to_goal, 0)

            # if it's shorter to turn in the other direction, we do it
            error_theta = pose_goal[2] - robot_state.current_pose[2]
            if abs(error_theta) > pi:
                if error_theta > 0:
                    error_theta -= 2*pi
                else:
                    error_theta += 2*pi

            self.vel_profile_theta.set_new_goal(error_theta, 0)

        # Compute distance to goal
        # distance_to_goal = ((pose_goal[0] - robot_state.current_pose[0])**2 + (pose_goal[1] - robot_state.current_pose[1])**2)**0.5
        
        mag = self.vel_profile_mag.compute_vel(None)
        theta = self.vel_profile_theta.compute_vel(None)

        cmd_vel = Vel()

        angle_vec_dir = atan2(pose_goal[1] - robot_state.current_pose[1], pose_goal[0] - robot_state.current_pose[0])
        
        cmd_vel.init_from_mag_ang(mag, angle_vec_dir, theta)

        print(cmd_vel)

        return cmd_vel


class TrapezoidalVelocityProfile:

    class ProfileState(Enum):
        ACCELERATION = 1
        FLAT = 2
        DECELERATION = 3


    def __init__(self, max_speed, max_acceleration):
        self.v_max = max_speed
        self.a_max = max_acceleration

        self.start_pos = None
        self.end_pos = None

        self.state = None

        self.t_start = None

        self.t_end_acc = None
        self.t_end_flat = None
        self.t_end_dec = None

        self.error_negative = False

    
    def set_new_goal(self, start_pos, goal_pos):

        self.start_pos = start_pos
        self.end_pos = goal_pos

        if self.start_pos > self.end_pos:
            self.error_negative = True

        tf = 2 * sqrt(abs(self.end_pos - self.start_pos) / self.a_max)

        midpoint_vel = self.a_max * tf / 2

        if midpoint_vel <= self.v_max: # 2 segments case
            self.t_end_acc = tf/2
            self.t_end_flat = None
            self.t_end_dec = tf
        
        else: # 3 segments case
            tf = abs(self.end_pos - self.start_pos) / self.v_max + self.v_max / self.a_max
            self.t_end_acc = self.v_max / self.a_max
            self.t_end_flat = tf - self.t_end_acc
            self.t_end_dec = tf

        self.t_start = time.time()

    
    def compute_vel(self, pos_current):

        t = time.time() - self.t_start

        # Update profile state
        if t < self.t_end_acc:
            self.state = self.ProfileState.ACCELERATION
        elif self.t_end_flat is not None and t < self.t_end_flat:
            self.state = self.ProfileState.FLAT
        elif t < self.t_end_dec:
            self.state = self.ProfileState.DECELERATION
        else:
            return 0

        # Update cmd
        cmd = 0
        if self.state == self.ProfileState.ACCELERATION:
            cmd = self.a_max * t
        elif self.state == self.ProfileState.FLAT:
            cmd = self.v_max
        else:
            if self.t_end_flat is not None:
                cmd = self.v_max - self.a_max * (t - self.t_end_flat)
            else:
                midpoint_vel = self.a_max * self.t_end_dec / 2
                cmd = midpoint_vel - self.a_max * (t - self.t_end_acc)
        
        if not self.error_negative:
            cmd = -cmd

        ic(self.state, t, self.t_end_acc, self.t_end_flat, self.t_end_dec, cmd)

        return cmd




        
class PoseControl:
    def __init__(self, viz=None):

        self.viz = viz
        
        # Objects instanciation

        self.robot_state = None
        self.cmd_vel_updater = CmdVelUpdater2()

        # Variables related to goals
        
        # Path to follow [[x, y, theta], ...]. The first pose must be the current robot pose.
        self.cmd_path = []
        # Current goal index in cmd_path
        self.i_goal = None
        # Convenience variable that is equal to cmd_path[i_goal]
        self.current_goal = None
        # Convenience variable that is equal to cmd_path[i_goal-1]
        self.prev_goal = None


    def set_robot_state(self, robot_state):
        self.robot_state = robot_state


    def set_cmd_path(self, cmd_path):
        """First pose must be the current robot pose."""
        if len(cmd_path) == 0:
            self.i_goal = None
            self.current_goal = None
            self.prev_goal = None
        else:
            self.cmd_path = cmd_path
            self.i_goal = 1
            self.current_goal = self.cmd_path[1]
            self.prev_goal = self.cmd_path[0]
    

    def control_loop_spin_once(self, dt):

        if self.i_goal is None:
            return Vel(0., 0., 0.).to_twist() # No cmd path to follow, return
        
        goal_reached = self.is_current_goal_reached()


        if goal_reached:
            self.switch_to_next_goal()
        
        # We reached the last pose of the cmd path
        if self.i_goal is None:
            return Vel(0., 0., 0.).to_twist()

        # Compute the command velocity
        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(self.robot_state, self.current_goal, dt)

        # Viz
        if self.viz is not None:
            # self.viz.draw_goal_poses(self.robot_state.current_pose, self.cmd_path)
            # self.viz.update()
            pass
        
        # publish the velocity (expressed in the base_link frame)
        cmd_vel = Vel.to_robot_frame(self.robot_state.current_pose, cmd_vel)    
        return cmd_vel.to_twist()


    def is_current_goal_reached(self):
        """Checks if the goal is reached and switch to the next one if it is the case.
        Should not be called if i_goal is None = no path to follow."""

        error_max_lin = 0.01
        error_max_ang = 0.01


        if abs(self.robot_state.current_pose[0] - self.current_goal[0]) < error_max_lin and abs(self.robot_state.current_pose[1] - self.current_goal[1]) < error_max_lin:
            if self.check_angle(self.robot_state.current_pose[2], self.current_goal[2], error_max_ang):
                return True
    
        return False


    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if (abs(2*pi-error) < 0.01):
            error = 0
        return error < error_max


    def switch_to_next_goal(self):
        """If the current goal wasn't the last one, switch to the next one by incrementing i_goal.
        Otherwise, clear the cmd_path and set i_goal to None."""

        self.i_goal += 1

        if self.i_goal == len(self.cmd_path):
            self.cmd_path = []
            self.i_goal = None
            self.current_goal = None
            self.prev_goal = None
            return
        
        self.current_goal = self.cmd_path[self.i_goal]
        self.prev_goal = self.cmd_path[self.i_goal-1]
        

    