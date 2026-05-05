"""

Visibility Road Map Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np

from champi_navigation.visibility_planner.geometry import Geometry
from champi_navigation.visibility_planner.dijkstra_search import DijkstraSearch


class VisibilityRoadMap:

    def __init__(self, expand_distance):
        self.expand_distance = expand_distance

    def planning(self, start_x, start_y, goal_x, goal_y, obstacles):

        nodes = self.generate_visibility_nodes(start_x, start_y,
                                               goal_x, goal_y, obstacles)

        expanded_obstacles = self.build_expanded_obstacles(obstacles)

        road_map_info = self.generate_road_map_info(nodes, expanded_obstacles)

        rx, ry = DijkstraSearch().search(
            start_x, start_y,
            goal_x, goal_y,
            [node.x for node in nodes],
            [node.y for node in nodes],
            road_map_info
        )

        return rx, ry

    def generate_visibility_nodes(self, start_x, start_y, goal_x, goal_y,
                                  obstacles):

        # add start and goal as nodes
        nodes = [DijkstraSearch.Node(start_x, start_y),
                 DijkstraSearch.Node(goal_x, goal_y, 0, None)]

        # add vertexes in configuration space as nodes
        for obstacle in obstacles:

            cvx_list, cvy_list = self.calc_vertexes_in_configuration_space(
                obstacle.x_list, obstacle.y_list)

            for (vx, vy) in zip(cvx_list, cvy_list):
                nodes.append(DijkstraSearch.Node(vx, vy))

        return nodes

    def calc_vertexes_in_configuration_space(self, x_list, y_list):
        x_list = x_list[0:-1]
        y_list = y_list[0:-1]
        cvx_list, cvy_list = [], []

        n_data = len(x_list)

        for index in range(n_data):
            offset_x, offset_y = self.calc_offset_xy(
                x_list[index - 1], y_list[index - 1],
                x_list[index], y_list[index],
                x_list[(index + 1) % n_data], y_list[(index + 1) % n_data],
            )
            cvx_list.append(offset_x)
            cvy_list.append(offset_y)

        return cvx_list, cvy_list

    def build_expanded_obstacles(self, obstacles):
        """Build expanded obstacle polygons using configuration space vertices."""
        expanded = []
        for obstacle in obstacles:
            cvx_list, cvy_list = self.calc_vertexes_in_configuration_space(
                obstacle.x_list, obstacle.y_list)
            expanded.append(ObstaclePolygon(cvx_list, cvy_list))
        return expanded

    def generate_road_map_info(self, nodes, obstacles):

        road_map_info_list = []

        for target_id, target_node in enumerate(nodes):
            road_map_info = []
            for node_id, node in enumerate(nodes):
                if node_id == target_id:
                    continue

                is_valid = True
                for obstacle in obstacles:
                    if not self.is_edge_valid(target_node, node, obstacle):
                        is_valid = False
                        break
                if is_valid:
                    road_map_info.append(node_id)

            road_map_info_list.append(road_map_info)

        return road_map_info_list

    @staticmethod
    def is_edge_valid(target_node, node, obstacle):
        eps = 1e-6
        skipped_any = False

        for i in range(len(obstacle.x_list) - 1):
            p1 = Geometry.Point(target_node.x, target_node.y)
            p2 = Geometry.Point(node.x, node.y)
            p3 = Geometry.Point(obstacle.x_list[i], obstacle.y_list[i])
            p4 = Geometry.Point(obstacle.x_list[i + 1], obstacle.y_list[i + 1])

            # Skip if an edge endpoint coincides with a polygon vertex
            # (C-space nodes lie on expanded polygon vertices)
            if (abs(p1.x - p3.x) < eps and abs(p1.y - p3.y) < eps) or \
               (abs(p1.x - p4.x) < eps and abs(p1.y - p4.y) < eps) or \
               (abs(p2.x - p3.x) < eps and abs(p2.y - p3.y) < eps) or \
               (abs(p2.x - p4.x) < eps and abs(p2.y - p4.y) < eps):
                skipped_any = True
                continue

            if Geometry.is_seg_intersect(p1, p2, p3, p4):
                return False

        # If we skipped some edges due to vertex coincidence, verify the edge
        # midpoint is not inside the polygon (prevents diagonal shortcuts)
        if skipped_any:
            mid_x = (target_node.x + node.x) / 2.0
            mid_y = (target_node.y + node.y) / 2.0
            if VisibilityRoadMap._point_in_polygon(mid_x, mid_y, obstacle):
                return False

        return True

    @staticmethod
    def _point_in_polygon(x, y, obstacle):
        """Ray casting point-in-polygon test."""
        n = len(obstacle.x_list) - 1  # closed polygon, skip last duplicate
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = obstacle.x_list[i], obstacle.y_list[i]
            xj, yj = obstacle.x_list[j], obstacle.y_list[j]
            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def calc_offset_xy(self, px, py, x, y, nx, ny):
        p_vec = math.atan2(y - py, x - px)
        n_vec = math.atan2(ny - y, nx - x)
        offset_vec = math.atan2(math.sin(p_vec) + math.sin(n_vec),
                                math.cos(p_vec) + math.cos(
                                    n_vec)) + math.pi / 2.0
        offset_x = x + self.expand_distance * math.cos(offset_vec)
        offset_y = y + self.expand_distance * math.sin(offset_vec)
        return offset_x, offset_y


class ObstaclePolygon:

    def __init__(self, x_list, y_list):
        self.x_list = x_list
        self.y_list = y_list

        self.close_polygon()
        self.make_clockwise()

    def make_clockwise(self):
        if not self.is_clockwise():
            self.x_list = list(reversed(self.x_list))
            self.y_list = list(reversed(self.y_list))

    def is_clockwise(self):
        n_data = len(self.x_list)
        eval_sum = sum([(self.x_list[i + 1] - self.x_list[i]) *
                        (self.y_list[i + 1] + self.y_list[i])
                        for i in range(n_data - 1)])
        eval_sum += (self.x_list[0] - self.x_list[n_data - 1]) * \
                    (self.y_list[0] + self.y_list[n_data - 1])
        return eval_sum >= 0

    def close_polygon(self):
        is_x_same = self.x_list[0] == self.x_list[-1]
        is_y_same = self.y_list[0] == self.y_list[-1]
        if is_x_same and is_y_same:
            return  # no need to close

        self.x_list.append(self.x_list[0])
        self.y_list.append(self.y_list[0])



if __name__ == '__main__':
    main()
