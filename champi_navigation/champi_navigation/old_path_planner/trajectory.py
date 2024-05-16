#!/usr/bin/env python3
from enum import Enum

class TRAJECTORY_TYPE(Enum):
    STRAIGHT = 1
    CIRCULAR = 2
    CLOTHOID = 3

class POSE_TYPE(Enum):
    OTHER = 0
    START = 1
    MID = 2
    END = 3

class Pose():
    def __init__(self, type, position, orientation) -> None:
        self.type:POSE_TYPE = type
        self.position_m:tuple[float, float] = position
        self.orientation_rad:float = orientation
        self.linear_speed:tuple[float, float] = (None, None)
        self.angular_speed:float = None
        self.linear_acceleration:tuple[float, float] = (None, None)
        self.angular_acceleration:float = None
        self.tolerance:float = None

class Trajectory(): 
    def __init__(self) -> None:
        self.__type:TRAJECTORY_TYPE = None
        self.__control_poses: list[Pose] = None
        self.__total_time:float = None

    def get_control_poses(self) -> list[Pose]:
        return self.__control_poses
    
    def get_next_poses(self, delta_distance=0.01, horizon_size=5) -> list[Pose]:
        """Generate next n poses of the trajectory, sampling a new pose every *delta_distance*
        # Interpolation should depend on the type of the trajectory. See at the end

        Args:
            delta_distance (float): _description_
            horizon_size (int): 

        Returns:
            list[Pose]: _description_
        """
        # j'aimerais d'abord update les control poses comme le world state a probablement changé,
        # jsp si c'est le moyen opti de le faire ici ???
        # trajectory_builder.update_trajectory(self) # update the control points depending on the world state

        # linear interpolation between two control poses to get the next poses
        if self.__type == TRAJECTORY_TYPE.STRAIGHT:
            # linear interpolation
            pass
        else:
            return None

        pass


class TrajectoryBuilder(): # Singleton
    def __init__(self, world_state) -> None:
        self.world_state = world_state

    def build(self, start, end, max_vel_along_this_path, max_acc_along_this_path) -> Trajectory:
        """Construct a trajectory based on the current state of the world

        Args:
            start (Point):
            end (Point):
            max_vel_along_this_path (float):
            max_acc_along_this_path (float):

        Returns:
            Trajectory:
        """
        trajectory = Trajectory() 
        #...

        # avoid the obstacles blablabla

        #...
        return trajectory
    
    def update_trajectory(self, trajectory) -> None:
        # update the given trajectory with the current world state
        # only update the control_poses
        pass
