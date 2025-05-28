#!/usr/bin/env python3

from __future__ import annotations  # Needed for type hinting in the class itself

from math import atan2, cos, sin, sqrt, pi
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point

from champi_libraries_py.utils.angles import normalize_angle, get_yaw


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
            self.theta = get_yaw(pose)

        elif point is not None:
            self.x = point.x
            self.y = point.y
            self.theta = 0.

        else:
            self.x = x
            self.y = y
            self.theta = theta

    def to_ros_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.orientation.z = sin(self.theta / 2)
        pose.orientation.w = cos(self.theta / 2)
        return pose
    

    def dist_to_line(self, start:Pose2D, end:Pose2D):
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


    def dist_to_line_signed(self, start:Pose2D, end:Pose2D):
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
    
    def position_strictly_equals(self, other:Pose2D):
        return self.x == other.x and self.y == other.y
    

    def get_angle_difference(self, other:Pose2D):
        """ Computes the MINIMAL angle difference with another pose given in argument.
        E.g if self.theta = 0 and other.theta = 6, the difference will be -0.28 rad not 6 rad

        Args:
            other (Pose2D): The other pose (only theta used)
        
        Returns:
            float: The angle difference (in radians) between the two poses
        """
        diff = other.theta - self.theta

        return normalize_angle(diff)
    
    
    def get_angle_difference_to_look_at(self, target:Pose2D, robot_angle_when_looking_at_point):
        diff = self.get_target_angle(target) - self.theta - robot_angle_when_looking_at_point
        
        return normalize_angle(diff)

    def get_target_angle(self, target:Pose2D):
        return atan2(target.y - self.y, target.x - self.x)
    
        
    def get_distance2(self, other:Pose2D):
        return (self.x - other.x)**2 + (self.y - other.y)**2
    

    def get_distance(self, other:Pose2D):
        return sqrt(self.get_distance2(other))


    def __str__(self):
        return f'Pose2D(x={self.x}, y={self.y}, theta={self.theta})'



class Vect2D:
    def __init__(self, x=0., y=0., pose2d: Pose2D=None):

        if pose2d is not None:
            self.x = pose2d.x
            self.y =pose2d.y
        else:
            self.x = x
            self.y = y

    def to_pose2d(self):
        return Pose2D(x=self.x, y=self.y)

    def mult(self, scalar: float):
        """Multiply the vector by a scalar."""
        return Vect2D(self.x * scalar, self.y * scalar)

    def div(self, scalar: float):
        """Divide the vector by a scalar."""
        if scalar == 0:
            raise ValueError("Division by zero is not allowed.")
        return Vect2D(self.x / scalar, self.y / scalar)

    def norm(self):
        """Compute the Euclidean norm of the vector."""
        return sqrt(self.x**2 + self.y**2)

    def sub(self, other: Vect2D):
        """Subtract another vector from this vector."""
        return Vect2D(self.x - other.x, self.y - other.y)

    def add(self, other: Vect2D):
        """Add another vector to this vector."""
        return Vect2D(self.x + other.x, self.y + other.y)

    def normalize(self):
        """Normalize the vector to have a magnitude of 1."""
        norm = self.norm()
        if norm == 0:
            raise ValueError("Cannot normalize a zero vector.")
        return self.div(norm)


    def angle(self, other: Vect2D):
        """Compute the angle between this vector and another vector in radians."""
        dot_product = self.x * other.x + self.y * other.y
        norm_product = self.norm() * other.norm()
        if norm_product == 0:
            raise ValueError("Cannot compute the angle with a zero vector.")
        return acos(dot_product / norm_product)