#!/usr/bin/env python3

from math import atan2, cos, sin, sqrt
import numpy as np
from geometry_msgs.msg import Twist


class Vel:
    """Velocity. It can be expressed in any frame."""
    def __init__(self, x=0., y=0., theta=0.):
        self.x = x # m/s
        self.y = y # m/s
        self.theta = theta # rad/s
    
    def init_from_mag_ang(self, mag, angle, theta):
        self.x = mag * cos(angle)
        self.y = mag * sin(angle)
        self.theta = theta

    def as_mag_ang(self):
        angle = atan2(self.y, self.x)
        mag = (self.x**2 + self.y**2)**0.5
        return np.array([mag, angle, self.theta])

    def __str__(self):
        return f'CmdVel(x={self.x}, y={self.y}, theta={self.theta})'

    def to_twist(self):
        twist = Twist()
        twist.linear.x = float(self.x)
        twist.linear.y = float(self.y)
        twist.angular.z = float(self.theta)
        return twist
    
    @staticmethod
    def to_robot_frame(robot_pose, cmd_vel):
        """Transform a velocity expressed in the base_link frame to the robot frame"""
        x = cmd_vel.x * cos(robot_pose[2]) + cmd_vel.y * sin(robot_pose[2])
        y = -cmd_vel.x * sin(robot_pose[2]) + cmd_vel.y * cos(robot_pose[2])
        theta = cmd_vel.theta
        return Vel(x, y, theta)
    
    @staticmethod
    def to_global_frame(robot_pose, cmd_vel):
        """Transform a velocity expressed in the robot frame to the base_link frame"""
        x = cmd_vel.x * cos(robot_pose[2]) - cmd_vel.y * sin(robot_pose[2])
        y = cmd_vel.x * sin(robot_pose[2]) + cmd_vel.y * cos(robot_pose[2])
        theta = cmd_vel.theta
        return Vel(x, y, theta)


class RobotState:
    def __init__(self, pose: np.array, vel: Vel):
        self.pose = pose  # a list of 3 elements [x, y, theta]
        self.vel = vel


class PathFollowParams:
    """This class is used to store the parameters of a CmdVelUpdater class.
    We need to pass this class each time we call compute_cmd_vel, so we can change the parameters easily.

    The parameters are:
    - Robot Pose
    - Segment Start Pose (note: the segment is the line that is currently being followed)
    - Segment End Pose (goal)
    - Arrival Speed (at goal)
    - Arrival Angle

    - max_speed_linear
    - max_speed_angular
    - max_acc_linear
    - max_acc_angular


    For segment start and end pose, only the x and y coordinates are used. The goal orientation is given by the
    arrival angle.

    """

    def __init__(self):
        self.robot_state = None  # type RobotState
        self.segment_start = None # type Pose
        self.segment_end = None # type Pose
        self.arrival_speed = None
        self.arrival_angle = None
        self.max_speed_linear = None
        self.max_speed_angular = None
        self.max_acc_linear = None
        self.max_acc_angular = None


def dist_point_to_line(point, line):
    x0, y0 = point
    x1, y1 = line[0]
    x2, y2 = line[1]
    return abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)


def dist_point_to_line_signed(point, line):
    x0, y0 = point
    x1, y1 = line[0]
    x2, y2 = line[1]
    return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)