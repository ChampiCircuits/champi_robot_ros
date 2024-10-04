#!/usr/bin/env python3

from math import atan2, cos, sin, sqrt, pi
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point
from champi_interfaces.msg import CtrlGoal


class Vel2D:
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
        x = cmd_vel.x * cos(robot_pose.theta) + cmd_vel.y * sin(robot_pose.theta)
        y = -cmd_vel.x * sin(robot_pose.theta) + cmd_vel.y * cos(robot_pose.theta)
        theta = cmd_vel.theta
        return Vel2D(x, y, theta)
    
    @staticmethod
    def to_global_frame(robot_pose, cmd_vel):
        """Transform a velocity expressed in the robot frame to the base_link frame"""
        x = cmd_vel.x * cos(robot_pose.theta) - cmd_vel.y * sin(robot_pose.theta)
        y = cmd_vel.x * sin(robot_pose.theta) + cmd_vel.y * cos(robot_pose.theta)
        theta = cmd_vel.theta
        return Vel2D(x, y, theta)


class Pose2D:
    def __init__(self, x=0., y=0., theta=0., pose:Pose=None, point:Point=None):

        # This assertion is here because it's easy to do mistakes with that constructor, e.g Pose2D(pose) instead of Pose2D(pose=pose)
        assert type(x) == float and type(y) == float and type(theta) == float, "x, y and theta must be floats"

        if pose is not None:
            self.x = pose.position.x
            self.y = pose.position.y
            self.theta = 2 * atan2(pose.orientation.z, pose.orientation.w)

        elif point is not None:
            self.x = point.x
            self.y = point.y
            self.theta = 0.

        else:
            self.x = x
            self.y = y
            self.theta = theta

    def to_ros_pose(self):
        print("pose2D :")
        print(self.x)
        print(self.y)
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.orientation.z = sin(self.theta / 2)
        pose.orientation.w = cos(self.theta / 2)
        return pose
    

    def dist_to_line(self, start, end):
        """Compute the distance between this point and a LINE (not segment!) defined by two points.

        Args:
            start (Pose2D): Start of the line
            end (Pose2D): Start of the line

        Returns:
            float: The distance between the point and the line
        """
        x0, y0 = self.x, self.y
        x1, y1 = start.x, start.y
        x2, y2 = end.x, end.y
        return abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)


    def dist_to_line_signed(self, start, end):
        """Compute the distance between this point and a LINE (not segment!) defined by two points.
        Positive if point is on one side of the line, negative if on the other side.

        Args:
            start (Pose2D): Start of the line
            end (Pose2D): Start of the line

        Returns:
            float: The distance between the point and the line
        """
        x0, y0 = self.x, self.y
        x1, y1 = start.x, start.y
        x2, y2 = end.x, end.y
        return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def position_equals(self, other):
        return self.x == other.x and self.y == other.y
    

    def get_angle_difference(self, other):
        """ Computes the MINIMAL angle difference with another pose given in argument.
        E.g if self.theta = 0 and other.theta = 6, the difference will be -0.28 rad not 6 rad

        Args:
            other (Pose2D): The other pose (only theta used)
        
        Returns:
            float: The angle difference (in radians) between the two poses
        """
        diff = other.theta - self.theta
        if abs(diff) > pi:
            if diff > 0:
                diff -= 2 * pi
            else:
                diff += 2 * pi

        return diff
    
    def get_angle_difference_to_look_at(self, target):
        diff = self.get_target_angle(target) - self.theta
        
        if abs(diff) > pi:
            if diff > 0:
                diff -= 2 * pi
            else:
                diff += 2 * pi
        
        return diff

    def get_target_angle(self, target):
        return atan2(target.y - self.y, target.x - self.x)
    
        
    def get_distance2(self, other):
        return (self.x - other.x)**2 + (self.y - other.y)**2
    

    def get_distance(self, other):
        return sqrt(self.get_distance2(other))


    def __str__(self):
        return f'Pose2D(x={self.x}, y={self.y}, theta={self.theta})' # TODO Comprendre pourquoi ça ne marche pas
    
    
    def to_string(self):
        return f'Pose2D(x={self.x}, y={self.y}, theta={self.theta})'  # TODO Enlever quand __str__ marchera


class RobotState:
    def __init__(self, pose: Pose2D, vel: Vel2D):
        self.pose = pose
        self.vel = vel

    def to_string(self):
        return f'RobotState(pose={self.pose}, vel={self.vel})'


class PathFollowParams:
    """This class is used to store the parameters of a CmdVelUpdater class.
    We need to pass this class each time we call compute_cmd_vel, so we can change the parameters easily.
    """

    def __init__(self, max_acc_linear:float, max_acc_angular:float):
        self.robot_state:RobotState = None
        self.segment_start:Pose2D = None  # (note: the segment is the line that is currently being followed)
        self.segment_end:Pose2D = None  # goal
        self.arrival_speed:float = None  # at goal
        self.max_speed_linear:float = None
        self.max_speed_angular:float = None
        self.max_acc_linear = max_acc_linear
        self.max_acc_angular = max_acc_angular
        self.look_at_point:Pose2D = None  # None means no look_at_point

    
    def update_ctrl_goal(self, robot_state:RobotState, ctrl_goal:CtrlGoal):
        """Compute all the attributs. This method is called each time a new ctrl_goal is received.

        Args:
            robot_state (RobotState): Current state of the robot.
            ctrl_goal (CtrlGoal): New CtrlGoal received.
        """
        self.robot_state = robot_state
        self.segment_start = robot_state.pose
        self.segment_end = Pose2D(pose=ctrl_goal.pose)
        self.arrival_speed = ctrl_goal.end_speed
        self.max_speed_linear = ctrl_goal.max_linear_speed
        self.max_speed_angular = ctrl_goal.max_angular_speed
        if ctrl_goal.do_look_at_point:
            self.look_at_point = Pose2D(point=ctrl_goal.look_at_point)
        else:
            self.look_at_point = None
    
    def to_string(self):
        return f"RobotState: {self.robot_state.to_string()},\n \
                Segment Start: {self.segment_start},\n \
                Segment End: {self.segment_end},\n \
                Arrival Speed: {self.arrival_speed},\n \
                Max Speed Linear: {self.max_speed_linear},\n \
                Max Speed Angular: {self.max_speed_angular},\n \
                Max Acc Linear: {self.max_acc_linear},\n \
                Max Acc Angular: {self.max_acc_angular}\n \
                Look At Point: {self.look_at_point}\n`\n"
                
