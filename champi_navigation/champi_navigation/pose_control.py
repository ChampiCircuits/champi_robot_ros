#!/usr/bin/env python3

from math import pi, atan2
from icecream import ic

from geometry_msgs.msg import Twist

from champi_navigation.utils import Vel, RobotState
from champi_navigation.pid import PID


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
        # self.pid_angle_vec_dir = PID(1, 0, 0)
        pass
    
    def compute_cmd_vel(self, robot_state, pose_goal, dt):
        # if it's shorter to turn in the other direction, we do it
        error_theta = pose_goal[2] - robot_state.current_pose[2]
        if abs(error_theta) > pi:
            if error_theta > 0:
                error_theta -= 2*pi
            else:
                error_theta += 2*pi
        
        cmd_vel = Vel()

        angle_vec_dir = atan2(pose_goal[1] - robot_state.current_pose[1], pose_goal[0] - robot_state.current_pose[0])

        linear_speed = 0.1
        angular_speed = 0.1

        if error_theta > 0:
            cmd_vel.theta = angular_speed
        else:
            cmd_vel.theta = -angular_speed
        
        cmd_vel.init_from_mag_ang(linear_speed, angle_vec_dir, cmd_vel.theta)

        return cmd_vel
        
class PoseControl:
    def __init__(self, viz=None):

        self.viz = viz
        
        # Objects instanciation

        self.robot_state = None
        self.cmd_vel_calc = CmdVelCalculator()

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
        cmd_vel = self.cmd_vel_calc.compute_cmd_vel(self.robot_state, self.current_goal, dt)

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

        error_max_lin = 1
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
        

    