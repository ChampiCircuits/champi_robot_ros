#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from math import sin, cos, atan2, hypot
import time
from enum import Enum
from icecream import ic

from astar import AStar
from bresenham import bresenham


class CostmapPathFinder(AStar):

    def __init__(self, width, height, m_per_pixel):
        # Costmap is a np array
        self.costmap = None
        self.width = width
        self.height = height

    def update_costmap(self, costmap):
        self.costmap = costmap

    def compute_path(self, start, goal, costmap):
        self.update_costmap(costmap)
        return self.astar(start, goal)

    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2
        return hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        (x1, y1) = n1
        (x2, y2) = n2
        return hypot(x2 - x1, y2 - y1)

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent and 4 diagonal neighbors that can be reached"""
        x, y = node

        # 8-connected grid
        neighbors = []
        if x > 0 and self.costmap[y][x - 1] == 0:
            neighbors.append((x - 1, y))
        if x < self.width - 1 and self.costmap[y][x + 1] == 0:
            neighbors.append((x + 1, y))
        if y > 0 and self.costmap[y - 1][x] == 0:
            neighbors.append((x, y - 1))
        if y < self.height - 1 and self.costmap[y + 1][x] == 0:
            neighbors.append((x, y + 1))
        if x > 0 and y > 0 and self.costmap[y - 1][x - 1] == 0:
            neighbors.append((x - 1, y - 1))
        if x < self.width - 1 and y > 0 and self.costmap[y - 1][x + 1] == 0:
            neighbors.append((x + 1, y - 1))
        if x > 0 and y < self.height - 1 and self.costmap[y + 1][x - 1] == 0:
            neighbors.append((x - 1, y + 1))
        if x < self.width - 1 and y < self.height - 1 and self.costmap[y + 1][x + 1] == 0:
            neighbors.append((x + 1, y + 1))

        return neighbors


class ComputePathResult(Enum):
    SUCCESS = 0
    START_NOT_IN_COSTMAP = 1
    GOAL_NOT_IN_COSTMAP = 2
    GOAL_IN_OCCUPIED_CELL = 3
    NO_PATH_FOUND = 4


def path_to_msg(path, angle=0.):

    """
    Always give an angle (unless you give an empty path, then the angle is not used)
    :param path:
    :param angle:
    :return:
    """

    # TODO remove parameter angle (path will have the angle for each point), and move this function to the node

    path_msg = Path()
    path_msg.header.frame_id = 'map'
    t = time.time()
    path_msg.header.stamp.sec = int(t)
    path_msg.header.stamp.nanosec = int((t - int(t)) * 1e9)

    if len(path) == 0:
        return path_msg

    for p in path:
        pose = PoseStamped()
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = 0.
        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = 0.
        pose.pose.orientation.w = 1.
        path_msg.poses.append(pose)

    # Set the latest pose orientation
    path_msg.poses[-1].pose.orientation.z = sin(angle / 2)
    path_msg.poses[-1].pose.orientation.w = cos(angle / 2)
    return path_msg


def is_line_free(start, end, costmap):
    for p in bresenham(int(start[0]), int(start[1]), int(end[0]), int(end[1])):
        if costmap[p[1], p[0]] != 0:
            return False

    return True


def optimize_path(path, costmap):
    current_start = path[0]

    new_path = [current_start]

    for current_end in path[1:]:
        if not is_line_free(current_start, current_end, costmap):
            new_path.append(current_end)
            current_start = current_end

    return new_path


class AStarPathPlanner:

    def __init__(self, width, height, m_per_pixel):

        self.m_per_pixel = m_per_pixel
        self.costmap_path_finder = CostmapPathFinder(width, height, m_per_pixel)

    def compute_path(self, robot_pose, goal_pose, costmap):

        start = self.m_to_pixel(robot_pose)
        goal = self.m_to_pixel(goal_pose)

        # If start or goal are not in the costmap, return
        if not (0 <= start[0] < costmap.shape[1] and 0 <= start[1] < costmap.shape[0]):
            return None, ComputePathResult.START_NOT_IN_COSTMAP
        if not (0 <= goal[0] < costmap.shape[1] and 0 <= goal[1] < costmap.shape[0]):
            path_msg = path_to_msg([])
            return path_msg, ComputePathResult.GOAL_NOT_IN_COSTMAP

        # If goal is in an occupied cell, return
        if costmap[goal[1], goal[0]] != 0:
            path_msg = path_to_msg([])
            return path_msg, ComputePathResult.GOAL_IN_OCCUPIED_CELL


        # If start is in an occupied cell, find the closest free cell
        if costmap[start[1], start[0]] != 0:
            # start = self.handle_start_in_occupied_cell_by_finding_closest_free_cell(start, costmap)
            start = self.handle_start_in_occupied_cell_by_clearing(start, costmap)
            if start is None:
                path_msg = path_to_msg([])
                return path_msg, ComputePathResult.NO_PATH_FOUND

        # If line is free, just go straight (no need to pathfind, we save time)
        if is_line_free(start, goal, costmap):
            path_msg = path_to_msg([robot_pose[:2], goal_pose[:2]], goal_pose[2])
            return path_msg, ComputePathResult.SUCCESS

        # Compute the path
        path = self.costmap_path_finder.compute_path(start, goal, costmap)

        if path is None: # No path found
            path_msg = path_to_msg([])
            return path_msg, ComputePathResult.NO_PATH_FOUND

        path = list(path)

        path = optimize_path(path, costmap)

        # Convert path to meters
        path_m = [self.pixel_to_m(p) for p in path]

        # Replace start and end with the actual start and end
        path_m[0] = robot_pose[:2]
        path_m[-1] = goal_pose[:2]  # goal is already in m

        path_msg = path_to_msg(path_m, goal_pose[2])

        return path_msg, ComputePathResult.SUCCESS

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


    def m_to_pixel(self, pos):
        return int(pos[0] / self.m_per_pixel), int(pos[1] / self.m_per_pixel)

    def pixel_to_m(self, pos):
        return pos[0] * self.m_per_pixel, pos[1] * self.m_per_pixel
