#!/usr/bin/env python3

import numpy as np
from bresenham import bresenham

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import diagnostic_msgs


from champi_navigation.costmap_astar import CostmapAStar
from champi_navigation.planning_feedback import ComputePathResult

from icecream import ic
import numpy as np


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


class PathPlanner:
    """Path planner that computes a path from a start pose to a goal pose, avoiding obstacles in a costmap.
    It uses uses the A* planner when obstacles are in the way, otherwise it creates a straight line to the goal.
    """

    def __init__(self):
        self.is_initialized = False
        # raw and optimized path stored for debug
        self.raw_path = []
        self.optimized_path = []
        
        # Latest result, for diagnostics
        self.latest_result = None
        
    def initialize(self, width: int, height: int, m_per_pixel: float):
        """Initializes the path planner.

        Args:
            width (int): Width of the costmap grid in pixels
            height (int): Height of the costmap grid in pixels
            m_per_pixel (float): Size in meters of a pixel in the costmap grid
        """
        
        self.is_initialized = True

        self.m_per_pixel = m_per_pixel
        # A* path pathfinder
        self.costmap_path_finder = CostmapAStar(width, height)


    def compute_path(self, robot_pose: Pose, goal_pose: Pose, costmap) -> tuple[list[Pose], ComputePathResult]:
        """Computes the path from robot_pose to goal_pose, avoiding obstacles in the costmap.

        Args:
            robot_pose (Pose): Robot's pose (only position is used)
            goal_pose (Pose): Goal's pose (only position is used)
            costmap (_type_): Costmap: 2D numpy array, where 0 is free space and 100 is occupied space

        Returns:
            tuple[list[Pose], ComputePathResult]: Path as a list of Pose, and the result of the computation.
            There is 2 success results: SUCCESS_STRAIGHT and SUCCESS_AVOIDANCE. But other results may also
            lead to a valid path: in case of GOAL_IN_OCCUPIED_CELL and NO_PATH_FOUND, a line straight to the goal,
            stopping at the first obstacle, is returned.
            So, the result is only for information. To know if a path could be computed, check if the returned path is not None.
        """
        
        # Reset raw and optimized path, that are used for debug
        self.raw_path = []
        self.optimized_path = []

        # Converts poses's positions to pixels
        start: tuple[int, int] = self.m_to_pixel(robot_pose)
        goal:  tuple[int, int] = self.m_to_pixel(goal_pose)

        # If start is not in the costmap, return
        if not (0 <= start[0] < costmap.shape[1] and 0 <= start[1] < costmap.shape[0]):
            self.latest_result = ComputePathResult.START_NOT_IN_COSTMAP
            return None, ComputePathResult.START_NOT_IN_COSTMAP
        
        # If goal is not in the costmap, return
        if not (0 <= goal[0] < costmap.shape[1] and 0 <= goal[1] < costmap.shape[0]):
            self.latest_result = ComputePathResult.GOAL_NOT_IN_COSTMAP
            return None, ComputePathResult.GOAL_NOT_IN_COSTMAP

        # If goal is in an occupied cell, no A* will be performed but we can still try to go straight until obstacle
        if costmap[goal[1], goal[0]] != 0:
            path_poses = self.compute_path_until_obstacle(robot_pose, goal_pose, costmap) # Note: returns None if no path found
            self.latest_result = ComputePathResult.GOAL_IN_OCCUPIED_CELL
            return path_poses, ComputePathResult.GOAL_IN_OCCUPIED_CELL
        
        # Call update_costmap() needed before costmap_path_finder.neighbors() or costmap_path_finder.astar()
        self.costmap_path_finder.update_costmap(costmap)

        # Handle the case where start is an occupied cell.
        # Why do we want to handle this situayion? Because sometimes the displacement of the robot is not very accurate;
        # and when it goes around an obstacle, sometimes it enters the obstacle a little bit.
        #
        # If start is in an occupied cell AND all 0 adjacent cells are occupied too, we need to solve that issue.
        # (the A* implementation handles the case where start cell is occupied but adjacent cells are free).
        #
        # Note: we almost never enter this block, but we keep it for safety
        #
        if costmap[start[1], start[0]] != 0 and self.costmap_path_finder.neighbors(start) == []:
            # start = self.handle_start_in_occupied_cell_by_finding_closest_free_cell(start, costmap)
            start = self.handle_start_in_occupied_cell_by_clearing(start, costmap)

        # If line is free, just go straight (no need to pathfind, we save time)
        if is_line_free(start, goal, costmap, check_start=True):
            path_poses = [robot_pose, goal_pose]
            self.latest_result = ComputePathResult.SUCCESS_STRAIGHT
            return path_poses, ComputePathResult.SUCCESS_STRAIGHT

        # Compute the path
        path = self.costmap_path_finder.astar(start, goal)

        # No path found
        if path is None:
            path_poses = self.compute_path_until_obstacle(robot_pose, goal_pose, costmap)
            self.latest_result = ComputePathResult.NO_PATH_FOUND
            return path_poses, ComputePathResult.NO_PATH_FOUND

        path = list(path)

        self.raw_path = path # for debug

        # Optimize path by removing intermediate points that are not necessary, and creating straight lines when possible
        path = optimize_path(path, costmap)

        self.optimized_path = path # for debug

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

        # Replace start and end with the actual start and end (because of the conversion to pixels, they can be a bit off)
        path_poses[0] = robot_pose
        path_poses[-1] = goal_pose

        self.latest_result = ComputePathResult.SUCCESS_AVOIDANCE
        return path_poses, ComputePathResult.SUCCESS_AVOIDANCE


    def find_closest_free_cell(self, start_row, start_col, costmap):
        free_cells = np.argwhere(costmap == 0)
        ic("free_cells", free_cells)

        if len(free_cells) == 0:
            return None

        distances = np.linalg.norm(free_cells - np.array([start_row, start_col]), axis=1)
        closest_idx = np.argmin(distances)
        closest = tuple(free_cells[closest_idx])
        ic("closest free cell", closest)
        return closest


    def handle_start_in_occupied_cell_by_clearing(self, start, costmap):
        """
        Trouve la cellule libre la plus proche, vide la cellule de départ, puis vide la ligne entre les deux.

        :param start: tuple (x, y) en index pixel (col, row)
        :param costmap: numpy 2D array (row, col)
        :return: nouvelle position de départ (col, row)
        """

        # ic("trigger handle start in occupied cell")
        start_col, start_row = round(start[0]), round(start[1])
        # ic("start_row", start_row, "start_col", start_col)

        target_row, target_col = self.find_closest_free_cell(start_row, start_col, costmap)
        # ic("target_row", target_row, "target_col", target_col)

        # Vider la cellule de départ
        if 0 <= start_row < costmap.shape[0] and 0 <= start_col < costmap.shape[1]:
            costmap[start_row, start_col] = 0

        # Tracer et vider la ligne entre start et cible
        for r, c in bresenham(start_row, start_col, target_row, target_col):
            if 0 <= r < costmap.shape[0] and 0 <= c < costmap.shape[1]:
                costmap[r, c] = 0

        return (target_col, target_row)

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


    def m_to_pixel(self, pose:Pose) -> tuple[int,int]:
        return int(pose.position.x / self.m_per_pixel), int(pose.position.y / self.m_per_pixel)


    def pixel_to_m(self, pos:tuple[int,int]) -> tuple[float, float]:
        # We need to add m_per_pixel/2, because we consider the center of the pixels and not the top-left corner.
        # Otherwise the path looks shifted by half a pixel.
        return pos[0] * self.m_per_pixel + self.m_per_pixel/2, pos[1] * self.m_per_pixel + self.m_per_pixel/2

    
    def get_raw_path_as_occupancy_grid(self):
        """For debug: We create an occupancy grid, and mark each cell mentionned in self.raw_path.

        :return (OccupancyGrid): OccupancyGrid message
        """

        width = self.costmap_path_finder.width
        height = self.costmap_path_finder.height

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.m_per_pixel
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height

        if self.raw_path == []:
            occupancy_grid.data = np.zeros((height, width), np.int8).flatten().tolist()
            return occupancy_grid
        
        data = np.zeros((height, width), dtype=np.int32)
        for p in self.raw_path:
            data[p[1], p[0]] = 50

        occupancy_grid.data = data.flatten().tolist()

        return occupancy_grid

    def get_optimized_path_as_occupancy_grid(self) -> OccupancyGrid:
        """For debug: We create an occupancy grid, and draw the given path all points of path with lines
        using bresenham algorithm.

        :return (OccupancyGrid): OccupancyGrid message
        """

        width = self.costmap_path_finder.width
        height = self.costmap_path_finder.height

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.m_per_pixel
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height

        if self.optimized_path == []:
            occupancy_grid.data = np.zeros((height, width), np.int8).flatten().tolist()
            return occupancy_grid
        
        data = np.zeros((height, width), dtype=np.int32)
        for i in range(1, len(self.optimized_path)):
            for p in bresenham(self.optimized_path[i-1][0], self.optimized_path[i-1][1],
                                self.optimized_path[i][0], self.optimized_path[i][1]):
                data[p[1], p[0]] = 70

        occupancy_grid.data = data.flatten().tolist()

        return occupancy_grid
    

    def compute_path_until_obstacle(self, robot_pose: Pose, goal_pose: Pose, costmap) -> list[Pose]:
        """
        Compute a line path until the first obstacle is found.
        Always call it when compute_path() returns NO_PATH_FOUND or GOAL_IN_OCCUPIED_CELL.

        :param robot_pose: Pose
        :param costmap: np array
        :return: list of Pose,
        """

        # Conversion to pixels
        start = self.m_to_pixel(robot_pose)
        goal = self.m_to_pixel(goal_pose)

        # Create breseham line from robot to goal
        line = list(bresenham(start[0], start[1], goal[0], goal[1]))

        # Browse the line pixels and the first obstacle
        end_of_line = None
        for i in range(1, len(line)):
            p = line[i]
            if costmap[p[1], p[0]] != 0:
                end_of_line = line[i-1]
                break

        if end_of_line is None or end_of_line == start:
            return None
        
        end_pose = Pose()
        end_pose.position.x, end_pose.position.y = self.pixel_to_m(end_of_line)
        # Next line: while waiting for the the goal to be free, at least it orients itself correctly.
        end_pose.orientation = goal_pose.orientation

        return [robot_pose, end_pose]
    

    def produce_diagnostics(self, stat):
        """Callback for the diagnostic updater to report the latest path computation result."""

        if not self.is_initialized:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Not initialized')
            stat.add('Latest ComputePathResult', 'N/A')

        if self.latest_result is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Initialized, no planning done yet')
            stat.add('Latest ComputePathResult', 'N/A')

        elif self.latest_result == ComputePathResult.SUCCESS_STRAIGHT or self.latest_result == ComputePathResult.SUCCESS_AVOIDANCE:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Initialized, planning OK")
            stat.add('Latest ComputePathResult', self.latest_result.name)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Initialized, planning NOK")
            stat.add('Latest ComputePathResult', self.latest_result.name)
    
        return stat
        


