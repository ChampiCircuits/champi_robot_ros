#!/usr/bin/env python3

from champi_libraries_py.data_types.geometry import Pose2D, Vel2D

class RobotState:
    def __init__(self, pose: Pose2D, vel: Vel2D):
        self.pose = pose
        self.vel = vel

    def to_string(self):
        return f'RobotState(pose={self.pose}, vel={self.vel})'