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

        expanded_obstacles = self.build_expanded_obstacles(obstacles)

        nodes = self._generate_visibility_nodes_from_expanded(
            start_x, start_y, goal_x, goal_y, expanded_obstacles)

        road_map_info = self._generate_road_map_info_vectorized(nodes, expanded_obstacles)

        rx, ry = DijkstraSearch().search(
            start_x, start_y,
            goal_x, goal_y,
            [node.x for node in nodes],
            [node.y for node in nodes],
            road_map_info
        )

        return rx, ry

    def _generate_visibility_nodes_from_expanded(self, start_x, start_y,
                                                  goal_x, goal_y,
                                                  expanded_obstacles):
        """Generate visibility nodes directly from already-expanded obstacles."""
        nodes = [DijkstraSearch.Node(start_x, start_y),
                 DijkstraSearch.Node(goal_x, goal_y, 0, None)]

        for obs in expanded_obstacles:
            # Expanded obstacle vertices are the C-space nodes (skip last closing vertex)
            for vx, vy in zip(obs.x_list[:-1], obs.y_list[:-1]):
                nodes.append(DijkstraSearch.Node(vx, vy))

        return nodes

    def _generate_road_map_info_vectorized(self, nodes, obstacles):
        """Fully vectorized road map generation using numpy broadcasting."""
        # Pre-compute all obstacle edges as numpy arrays
        all_edges_p3x = []
        all_edges_p3y = []
        all_edges_p4x = []
        all_edges_p4y = []
        edge_obstacle_idx = []

        for obs_idx, obstacle in enumerate(obstacles):
            for i in range(len(obstacle.x_list) - 1):
                all_edges_p3x.append(obstacle.x_list[i])
                all_edges_p3y.append(obstacle.y_list[i])
                all_edges_p4x.append(obstacle.x_list[i + 1])
                all_edges_p4y.append(obstacle.y_list[i + 1])
                edge_obstacle_idx.append(obs_idx)

        p3x = np.array(all_edges_p3x)  # (E,)
        p3y = np.array(all_edges_p3y)
        p4x = np.array(all_edges_p4x)
        p4y = np.array(all_edges_p4y)
        edge_obs_idx = np.array(edge_obstacle_idx)
        n_edges = len(p3x)

        # Extract node coordinates
        node_x = np.array([n.x for n in nodes])
        node_y = np.array([n.y for n in nodes])
        n_nodes = len(nodes)

        eps = 1e-6

        # Build all node pairs (i, j) where i != j
        # For pair (i, j): target = node[i], other = node[j]
        # We'll use broadcasting: pairs shape (n_pairs,), edges shape (E,)
        # Combined: (n_pairs, E)
        pair_i, pair_j = np.where(~np.eye(n_nodes, dtype=bool))
        n_pairs = len(pair_i)

        # Target and other node coords: (n_pairs,)
        tx = node_x[pair_i]
        ty = node_y[pair_i]
        nx = node_x[pair_j]
        ny = node_y[pair_j]

        # Reshape for broadcasting: (n_pairs, 1) vs (1, E) -> (n_pairs, E)
        tx_2d = tx[:, None]
        ty_2d = ty[:, None]
        nx_2d = nx[:, None]
        ny_2d = ny[:, None]
        p3x_2d = p3x[None, :]
        p3y_2d = p3y[None, :]
        p4x_2d = p4x[None, :]
        p4y_2d = p4y[None, :]

        # Check coincidence: (n_pairs, E)
        coincide = ((np.abs(tx_2d - p3x_2d) < eps) & (np.abs(ty_2d - p3y_2d) < eps)) | \
                   ((np.abs(tx_2d - p4x_2d) < eps) & (np.abs(ty_2d - p4y_2d) < eps)) | \
                   ((np.abs(nx_2d - p3x_2d) < eps) & (np.abs(ny_2d - p3y_2d) < eps)) | \
                   ((np.abs(nx_2d - p4x_2d) < eps) & (np.abs(ny_2d - p4y_2d) < eps))

        # Orientations: (n_pairs, E)
        # o1 = orientation(target, other, p3)
        o1 = (ny_2d - ty_2d) * (p3x_2d - nx_2d) - (nx_2d - tx_2d) * (p3y_2d - ny_2d)
        # o2 = orientation(target, other, p4)
        o2 = (ny_2d - ty_2d) * (p4x_2d - nx_2d) - (nx_2d - tx_2d) * (p4y_2d - ny_2d)
        # o3 = orientation(p3, p4, target)
        o3 = (p4y_2d - p3y_2d) * (tx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ty_2d - p4y_2d)
        # o4 = orientation(p3, p4, other)
        o4 = (p4y_2d - p3y_2d) * (nx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ny_2d - p4y_2d)

        s1 = np.sign(o1)
        s2 = np.sign(o2)
        s3 = np.sign(o3)
        s4 = np.sign(o4)

        # General intersection
        general_intersect = (s1 != s2) & (s3 != s4)

        # Collinear cases
        min_tx_nx = np.minimum(tx_2d, nx_2d)
        max_tx_nx = np.maximum(tx_2d, nx_2d)
        min_ty_ny = np.minimum(ty_2d, ny_2d)
        max_ty_ny = np.maximum(ty_2d, ny_2d)
        min_p3x_p4x = np.minimum(p3x_2d, p4x_2d)
        max_p3x_p4x = np.maximum(p3x_2d, p4x_2d)
        min_p3y_p4y = np.minimum(p3y_2d, p4y_2d)
        max_p3y_p4y = np.maximum(p3y_2d, p4y_2d)

        col1 = (s1 == 0) & (p3x_2d >= min_tx_nx) & (p3x_2d <= max_tx_nx) & \
                (p3y_2d >= min_ty_ny) & (p3y_2d <= max_ty_ny)
        col2 = (s2 == 0) & (p4x_2d >= min_tx_nx) & (p4x_2d <= max_tx_nx) & \
                (p4y_2d >= min_ty_ny) & (p4y_2d <= max_ty_ny)
        col3 = (s3 == 0) & (tx_2d >= min_p3x_p4x) & (tx_2d <= max_p3x_p4x) & \
                (ty_2d >= min_p3y_p4y) & (ty_2d <= max_p3y_p4y)
        col4 = (s4 == 0) & (nx_2d >= min_p3x_p4x) & (nx_2d <= max_p3x_p4x) & \
                (ny_2d >= min_p3y_p4y) & (ny_2d <= max_p3y_p4y)

        # An edge intersects a polygon edge if it's not coincident and intersects
        intersects = (~coincide) & (general_intersect | col1 | col2 | col3 | col4)

        # A pair is blocked if ANY non-coincident edge intersects
        pair_blocked_by_intersection = np.any(intersects, axis=1)  # (n_pairs,)

        # For pairs with coincident edges that are not already blocked,
        # check midpoint-in-polygon
        has_coincidence = np.any(coincide, axis=1)  # (n_pairs,)
        needs_midpoint_check = has_coincidence & ~pair_blocked_by_intersection

        # Compute midpoints for pairs that need checking
        pair_blocked_by_midpoint = np.zeros(n_pairs, dtype=bool)
        midpoint_indices = np.where(needs_midpoint_check)[0]

        if len(midpoint_indices) > 0:
            mid_x = (tx[midpoint_indices] + nx[midpoint_indices]) / 2.0
            mid_y = (ty[midpoint_indices] + ny[midpoint_indices]) / 2.0

            for k, pair_idx in enumerate(midpoint_indices):
                # Get obstacle indices that had coincident edges for this pair
                coincident_edges = coincide[pair_idx]
                obs_indices = np.unique(edge_obs_idx[coincident_edges])
                for obs_i in obs_indices:
                    if VisibilityRoadMap._point_in_polygon(mid_x[k], mid_y[k], obstacles[obs_i]):
                        pair_blocked_by_midpoint[pair_idx] = True
                        break

        # Final validity
        pair_valid = ~pair_blocked_by_intersection & ~pair_blocked_by_midpoint

        # Build adjacency list
        road_map_info_list = [[] for _ in range(n_nodes)]
        valid_indices = np.where(pair_valid)[0]
        for idx in valid_indices:
            road_map_info_list[pair_i[idx]].append(int(pair_j[idx]))

        return road_map_info_list

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
