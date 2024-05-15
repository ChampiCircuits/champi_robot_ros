#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry

from scipy.ndimage import gaussian_filter

from math import sin, cos, pi, sqrt, atan2, hypot
from icecream import ic
import numpy as np

from astar import AStar

import time

from scipy.interpolate import CubicSpline
import numpy as np

from bresenham import bresenham
from rdp import rdp

class CostmapPathFinder(AStar):

    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, width, height):
        # Costmap is a np array
        self.costmap = None
        self.width = width
        self.height = height

    def update_costmap(self, costmap):
        self.costmap = costmap

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

class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)


        self.timer = self.create_timer(timer_period_sec=0.2,
                                       callback=self.timer_callback)

        self.robot_pos = None
        self.goal = None

        self.m_per_pixel = None

        self.costmap_path_finder = None


    def odom_callback(self, msg):
        self.robot_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])

    def costmap_callback(self, msg):

        if self.costmap_path_finder is None:
            self.m_per_pixel = msg.info.resolution
            self.costmap_path_finder = CostmapPathFinder(msg.info.width, msg.info.height)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def timer_callback(self):

        if self.robot_pos is None or self.goal is None or self.costmap is None:
            return

        start = self.m_to_pixel(self.robot_pos)
        start = (int(start[0]), int(start[1]))
        goal = self.m_to_pixel(self.goal)
        goal = (int(goal[0]), int(goal[1]))

        self.costmap_path_finder.update_costmap(self.costmap)
        t_start = time.time()
        path = self.costmap_path_finder.astar(start, goal)

        if path is None:
            self.get_logger().warn('No path found (time elapsed: {:.3f}s)'.format(time.time() - t_start))
            return

        path = list(path)

        path = self.optimize_path(path)

        path_m = [self.pixel_to_m(p) for p in path]

        # Replace start and end with the actual start and end
        path_m[0] = self.robot_pos
        path_m[-1] = self.goal

        # path_m = self.smooth_path_rdp(path_m, 0.1)

        path_msg = self.path_to_msg(path_m)
        self.path_pub.publish(path_msg)

        # path_m = self.cubic_spline_interpolation(path_m)
        #
        self.get_logger().info(f'Path found in {time.time() - t_start:.3f}s')

    def path_to_msg(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
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

        return path_msg


    def is_line_free(self, start, end):
        for p in bresenham(int(start[0]), int(start[1]), int(end[0]), int(end[1])):
            if self.costmap[p[1], p[0]] != 0:
                return False

        return True

    def optimize_path(self, path):
        current_start = path[0]

        new_path = [current_start]


        for current_end in path[1:]:
            if not self.is_line_free(current_start, current_end):
                new_path.append(current_end)
                current_start = current_end

        return new_path

    def m_to_pixel(self, pos):
        return pos / self.m_per_pixel

    def pixel_to_m(self, pos):
        return (pos[0] * self.m_per_pixel, pos[1] * self.m_per_pixel)

def main(args=None):
    rclpy.init(args=args)
    planner = PlannerNode()
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()