#!/usr/bin/env python3

import numpy as np
from enum import Enum
from bresenham import bresenham

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid

from champi_navigation.costmap_astar import CostmapAStar


class ComputePathResult(Enum):
    SUCCESS_STRAIGHT = 0
    START_NOT_IN_COSTMAP = 1
    GOAL_NOT_IN_COSTMAP = 2
    GOAL_IN_OCCUPIED_CELL = 3
    NO_PATH_FOUND = 4
    SUCCESS_AVOIDANCE = 5


def is_line_free(start:tuple[int,int], end:tuple[int,int], costmap, check_start:bool) -> bool:
    for p in bresenham(int(start[0]), int(start[1]), int(end[0]), int(end[1])):
        if check_start and p == start:
            continue
        elif costmap[p[1], p[0]] != 0:
            return False

    return True


def optimize_path(path, costmap):
    current_start = path[0]

    new_path = [current_start]

    # Because of the A* implementation, the first point of path can be occupied. Which makes the A* more reliable,
    # but as a consequence, we need to ignore the first oiunt of the path (argument check_start=True of is_line_free())

    for i in range(1, len(path)):
        current_end = path[i]
        if not is_line_free(current_start, current_end, costmap, check_start=(i==1)):
            new_path.append(path[i-1])
            current_start = path[i-1]
    
    new_path.append(path[-1])
    
    return new_path


class AStarPathPlanner:

    def __init__(self, width, height, m_per_pixel):

        self.m_per_pixel = m_per_pixel
        self.costmap_path_finder = CostmapAStar(width, height)

        # raw and optimized path stored for debug
        self.raw_path = []
        self.optimized_path = []


    def compute_path(self, robot_pose: Pose, goal_pose: Pose, costmap) -> tuple[list[Pose], ComputePathResult]:

        start: tuple[int, int] = self.m_to_pixel(robot_pose)
        goal:  tuple[int, int] = self.m_to_pixel(goal_pose)

        # If start or goal are not in the costmap, return
        if not (0 <= start[0] < costmap.shape[1] and 0 <= start[1] < costmap.shape[0]):
            self.raw_path = []
            self.optimized_path = []
            return None, ComputePathResult.START_NOT_IN_COSTMAP
        if not (0 <= goal[0] < costmap.shape[1] and 0 <= goal[1] < costmap.shape[0]):
            path_poses = []
            self.raw_path = []
            self.optimized_path = []
            return path_poses, ComputePathResult.GOAL_NOT_IN_COSTMAP

        # If goal is in an occupied cell, return
        if costmap[goal[1], goal[0]] != 0:
            path_poses = []
            return path_poses, ComputePathResult.GOAL_IN_OCCUPIED_CELL


        # If start is in an occupied cell AND all 0 adjacent cells are occupied too, find the closest free cell
        # (the A* implementation handles the case where start cell is occupied but adjacent cells are free)
        # Note: we almost never enter this block, but we keep it for safety
        if costmap[start[1], start[0]] != 0 and self.costmap_path_finder.neighbors(start) == []:
            # start = self.handle_start_in_occupied_cell_by_finding_closest_free_cell(start, costmap)
            start = self.handle_start_in_occupied_cell_by_clearing(start, costmap)
            if start is None:
                path_poses = []
                return path_poses, ComputePathResult.NO_PATH_FOUND

        # If line is free, just go straight (no need to pathfind, we save time)
        if is_line_free(start, goal, costmap, check_start=True):
            path_poses = [robot_pose, goal_pose]
            self.raw_path = []
            self.optimized_path = []
            return path_poses, ComputePathResult.SUCCESS_STRAIGHT

        # Compute the path
        path = self.costmap_path_finder.compute_path(start, goal, costmap)

        if path is None: # No path found
            path_poses = []
            self.raw_path = []
            self.optimized_path = []
            return path_poses, ComputePathResult.NO_PATH_FOUND

        path = list(path)

        self.raw_path = path

        path = optimize_path(path, costmap)

        self.optimized_path = path

        # Convert path to meters
        path_m = [self.pixel_to_m(p) for p in path]

        # Create pose from points. Use orientation from the goal
        path_poses = []
        for p in path_m:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation = goal_pose.orientation
            path_poses.append(pose)

        # Replace start and end with the actual start and end
        path_poses[0] = robot_pose
        path_poses[-1] = goal_pose  # goal is already in m

        return path_poses, ComputePathResult.SUCCESS_AVOIDANCE


    def handle_start_in_occupied_cell_by_clearing(self, start, costmap):
        """
        Clears a circle of radius 0.1m around the start cell, to allow pathfinding.

        This approach is less conservative than finding the closest free cell, but the current prefers it
        (smooth motion).

        :param start: tuple (x, y)
        :param costmap: np array
        :return: None
        """

        start_int = (int(start[0]), int(start[1]))
        radius = int(0.1 / self.m_per_pixel)
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if 0 <= start_int[0] + i < costmap.shape[1] and 0 <= start_int[1] + j < costmap.shape[0]:
                    costmap[start_int[1] + j, start_int[0] + i] = 0

        return start


    def handle_start_in_occupied_cell_by_finding_closest_free_cell(self, start, costmap):
        """
        Changes the start into the closest free cell to the start cell, for when the start cell is occupied, preventing
        from performing A Star.

        This approach is more conservative than clearing a circle around the start cell, because it makes the robot
        stay further from obstacles. But the because of the PID correction of the current controller, the motion
        is less smooth.

        :param start: tuple (x, y)
        :param costmap: np array
        :return: tuple (x, y)
        """

        # 8-connected grid: we search in a circle of increasing radius
        max_radius = int(0.5 / self.m_per_pixel)
        radius = 1
        while radius < max_radius:
            for i in range(-radius, radius + 1):
                for j in range(-radius, radius + 1):
                    if 0 <= start[0] + i < costmap.shape[1] and 0 <= start[1] + j < costmap.shape[0]:
                        if costmap[start[1] + j, start[0] + i] == 0:
                            return start[0] + i, start[1] + j
            radius += 1

        return None


    def m_to_pixel(self, pose:Pose):
        return int(pose.position.x / self.m_per_pixel), int(pose.position.y / self.m_per_pixel)


    def pixel_to_m(self, pos):
        # return pos[0] * self.m_per_pixel, pos[1] * self.m_per_pixel
        return pos[0] * self.m_per_pixel + self.m_per_pixel/2, pos[1] * self.m_per_pixel + self.m_per_pixel/2

    
    def get_raw_path_as_occupancy_grid(self):
        """For debug"""

        width = self.costmap_path_finder.width
        height = self.costmap_path_finder.height

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.m_per_pixel
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height

        if self.raw_path == []:
            occupancy_grid.data = np.zeros((height, width)).flatten().tolist()
            return occupancy_grid
        
        data = np.zeros((height, width), dtype=np.int32)
        for p in self.raw_path:
            data[p[1], p[0]] = 50

        occupancy_grid.data = data.flatten().tolist()

        return occupancy_grid

    def get_path_as_occupancy_grid_bresenham(self, path):
        """For debug"""

        width = self.costmap_path_finder.width
        height = self.costmap_path_finder.height

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.m_per_pixel
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height

        if self.optimized_path == []:
            occupancy_grid.data = np.zeros((height, width)).flatten().tolist()
            return occupancy_grid
        
        data = np.zeros((height, width), dtype=np.int32)
        for i in range(1, len(self.optimized_path)):
            for p in bresenham(path[i-1][0], path[i-1][1], path[i][0], path[i][1]):
                data[p[1], p[0]] = 70

        occupancy_grid.data = data.flatten().tolist()

        return occupancy_grid
