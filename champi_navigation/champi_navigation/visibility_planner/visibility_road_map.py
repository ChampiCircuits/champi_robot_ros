"""

Visibility Road Map Planner

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import time
import numpy as np

from champi_navigation.visibility_planner.dijkstra_search import DijkstraSearch


class VisibilityRoadMap:

    def __init__(self, expand_distance):
        self.expand_distance = expand_distance
        self._static_cache = None

    def planning(self, start_x, start_y, goal_x, goal_y, static_obstacles, dynamic_obstacles=None):
        """Plan a path using a cached static road map + incremental dynamic edges.

        Args:
            static_obstacles: obstacles that rarely change (borders, zones). Road map is cached.
            dynamic_obstacles: obstacles that change every call (enemy robot). Always recomputed.

        Returns:
            (rx, ry, planning_ms): path coordinates and total computation time in milliseconds.
        """
        t_start = time.perf_counter()
        if dynamic_obstacles is None:
            dynamic_obstacles = []

        # Check / rebuild static cache
        key = self._cache_key(static_obstacles)
        if self._static_cache is None or self._static_cache['key'] != key:
            static_expanded = self.build_expanded_obstacles(static_obstacles)
            self._build_static_cache(static_obstacles, static_expanded)

        cache = self._static_cache
        static_nodes    = cache['nodes']
        static_adj      = cache['adj']
        static_expanded = cache['expanded_obstacles']

        # Expand dynamic obstacles and build their nodes
        dynamic_expanded = self.build_expanded_obstacles(dynamic_obstacles)
        dynamic_nodes = [
            DijkstraSearch.Node(start_x, start_y),
            DijkstraSearch.Node(goal_x, goal_y, 0, None),
        ]
        for obs in dynamic_expanded:
            for vx, vy in zip(obs.x_list[:-1], obs.y_list[:-1]):
                dynamic_nodes.append(DijkstraSearch.Node(vx, vy))
        N_d = len(dynamic_nodes)

        # Combined list: dynamic first (indices 0..N_d-1), then static (N_d..)
        all_nodes    = dynamic_nodes + static_nodes
        all_expanded = static_expanded + dynamic_expanded

        road_map_info = self._compute_incremental_road_map(
            all_nodes, N_d, static_adj, cache, all_expanded, dynamic_expanded
        )

        rx, ry = DijkstraSearch().search(
            start_x, start_y,
            goal_x, goal_y,
            [node.x for node in all_nodes],
            [node.y for node in all_nodes],
            road_map_info
        )

        planning_ms = (time.perf_counter() - t_start) * 1000.0
        return rx, ry, planning_ms

    def _cache_key(self, obstacles):
        """Hashable key derived from all obstacle coordinates."""
        coords = []
        for obs in obstacles:
            coords.extend(obs.x_list)
            coords.extend(obs.y_list)
        return tuple(coords)

    def _build_static_cache(self, static_obstacles, static_expanded):
        """Compute the full static-only road map and store it."""
        # Nodes: all C-space vertices (no start/goal)
        nodes = []
        for obs in static_expanded:
            for vx, vy in zip(obs.x_list[:-1], obs.y_list[:-1]):
                nodes.append(DijkstraSearch.Node(vx, vy))

        adj = self._generate_road_map_info_vectorized(nodes, static_expanded)

        # Flatten edge arrays once and store
        p3x, p3y, p4x, p4y, eidx = [], [], [], [], []
        for obs_idx, obs in enumerate(static_expanded):
            for i in range(len(obs.x_list) - 1):
                p3x.append(obs.x_list[i])
                p3y.append(obs.y_list[i])
                p4x.append(obs.x_list[i + 1])
                p4y.append(obs.y_list[i + 1])
                eidx.append(obs_idx)

        self._static_cache = {
            'key':                self._cache_key(static_obstacles),
            'expanded_obstacles': static_expanded,
            'nodes':              nodes,
            'adj':                adj,
            'p3x':                np.array(p3x),
            'p3y':                np.array(p3y),
            'p4x':                np.array(p4x),
            'p4y':                np.array(p4y),
            'edge_obs_idx':       np.array(eidx, dtype=int),
        }

    def _compute_incremental_road_map(self, all_nodes, N_d, static_adj, cache,
                                       all_expanded_obs, dynamic_expanded):
        """Build the full adjacency list for all_nodes.

        Static-static edges come directly from cache (O(N_s) list copy).
        Only edges involving at least one dynamic node are recomputed (O(N_d * N_total * E)).
        """
        N_total = len(all_nodes)

        # --- Seed from cached static-static adjacency (re-indexed by +N_d) ---
        t0 = time.perf_counter()
        road_map = [[] for _ in range(N_total)]
        for s_i, neighbors in enumerate(static_adj):
            road_map[N_d + s_i] = [N_d + j for j in neighbors]

        # Combine cached static edge arrays with fresh dynamic edge arrays
        n_static_obs = len(cache['expanded_obstacles'])
        dyn_p3x, dyn_p3y, dyn_p4x, dyn_p4y, dyn_eidx = [], [], [], [], []
        for obs_idx, obs in enumerate(dynamic_expanded):
            for i in range(len(obs.x_list) - 1):
                dyn_p3x.append(obs.x_list[i])
                dyn_p3y.append(obs.y_list[i])
                dyn_p4x.append(obs.x_list[i + 1])
                dyn_p4y.append(obs.y_list[i + 1])
                dyn_eidx.append(n_static_obs + obs_idx)

        if dyn_p3x:
            p3x = np.concatenate([cache['p3x'], np.array(dyn_p3x)])
            p3y = np.concatenate([cache['p3y'], np.array(dyn_p3y)])
            p4x = np.concatenate([cache['p4x'], np.array(dyn_p4x)])
            p4y = np.concatenate([cache['p4y'], np.array(dyn_p4y)])
            edge_obs_idx = np.concatenate([cache['edge_obs_idx'], np.array(dyn_eidx, dtype=int)])
        else:
            p3x, p3y = cache['p3x'], cache['p3y']
            p4x, p4y = cache['p4x'], cache['p4y']
            edge_obs_idx = cache['edge_obs_idx']

        # --- Re-check cached static-static edges against dynamic obstacles ---
        # The cache was built without dynamic obstacles, so any cached edge that
        # now crosses a dynamic obstacle must be removed.
        node_x = np.array([n.x for n in all_nodes])
        node_y = np.array([n.y for n in all_nodes])

        if dyn_p3x:
            dp3x = np.array(dyn_p3x); dp3y = np.array(dyn_p3y)
            dp4x = np.array(dyn_p4x); dp4y = np.array(dyn_p4y)
            d_eidx = np.array(dyn_eidx, dtype=int)

            # Collect all currently cached valid static-static pairs
            ss_i_list, ss_j_list = [], []
            for s_i, neighbors in enumerate(static_adj):
                for s_j in neighbors:
                    ss_i_list.append(N_d + s_i)
                    ss_j_list.append(N_d + s_j)

            if ss_i_list:
                ss_i = np.array(ss_i_list); ss_j = np.array(ss_j_list)
                tx_ss = node_x[ss_i]; ty_ss = node_y[ss_i]
                nx_ss = node_x[ss_j]; ny_ss = node_y[ss_j]

                tx_2d = tx_ss[:, None]; ty_2d = ty_ss[:, None]
                nx_2d = nx_ss[:, None]; ny_2d = ny_ss[:, None]
                p3x_2d = dp3x[None, :]; p3y_2d = dp3y[None, :]
                p4x_2d = dp4x[None, :]; p4y_2d = dp4y[None, :]

                eps = 1e-6
                coincide_ss = \
                    ((np.abs(tx_2d - p3x_2d) < eps) & (np.abs(ty_2d - p3y_2d) < eps)) | \
                    ((np.abs(tx_2d - p4x_2d) < eps) & (np.abs(ty_2d - p4y_2d) < eps)) | \
                    ((np.abs(nx_2d - p3x_2d) < eps) & (np.abs(ny_2d - p3y_2d) < eps)) | \
                    ((np.abs(nx_2d - p4x_2d) < eps) & (np.abs(ny_2d - p4y_2d) < eps))

                o1 = (ny_2d - ty_2d) * (p3x_2d - nx_2d) - (nx_2d - tx_2d) * (p3y_2d - ny_2d)
                o2 = (ny_2d - ty_2d) * (p4x_2d - nx_2d) - (nx_2d - tx_2d) * (p4y_2d - ny_2d)
                o3 = (p4y_2d - p3y_2d) * (tx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ty_2d - p4y_2d)
                o4 = (p4y_2d - p3y_2d) * (nx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ny_2d - p4y_2d)
                s1 = np.sign(o1); s2 = np.sign(o2); s3 = np.sign(o3); s4 = np.sign(o4)

                general_intersect = (s1 != s2) & (s3 != s4)
                min_tx_nx = np.minimum(tx_2d, nx_2d); max_tx_nx = np.maximum(tx_2d, nx_2d)
                min_ty_ny = np.minimum(ty_2d, ny_2d); max_ty_ny = np.maximum(ty_2d, ny_2d)
                min_p3x_p4x = np.minimum(p3x_2d, p4x_2d); max_p3x_p4x = np.maximum(p3x_2d, p4x_2d)
                min_p3y_p4y = np.minimum(p3y_2d, p4y_2d); max_p3y_p4y = np.maximum(p3y_2d, p4y_2d)
                col1 = (s1 == 0) & (p3x_2d >= min_tx_nx) & (p3x_2d <= max_tx_nx) & (p3y_2d >= min_ty_ny) & (p3y_2d <= max_ty_ny)
                col2 = (s2 == 0) & (p4x_2d >= min_tx_nx) & (p4x_2d <= max_tx_nx) & (p4y_2d >= min_ty_ny) & (p4y_2d <= max_ty_ny)
                col3 = (s3 == 0) & (tx_2d >= min_p3x_p4x) & (tx_2d <= max_p3x_p4x) & (ty_2d >= min_p3y_p4y) & (ty_2d <= max_p3y_p4y)
                col4 = (s4 == 0) & (nx_2d >= min_p3x_p4x) & (nx_2d <= max_p3x_p4x) & (ny_2d >= min_p3y_p4y) & (ny_2d <= max_p3y_p4y)

                intersects_ss = (~coincide_ss) & (general_intersect | col1 | col2 | col3 | col4)
                blocked_by_int = np.any(intersects_ss, axis=1)
                has_coin = np.any(coincide_ss, axis=1)
                needs_mid = has_coin & ~blocked_by_int

                blocked_by_mid = np.zeros(len(ss_i_list), dtype=bool)
                for k in np.where(needs_mid)[0]:
                    mid_x = (tx_ss[k] + nx_ss[k]) / 2.0
                    mid_y = (ty_ss[k] + ny_ss[k]) / 2.0
                    for obs_i in np.unique(d_eidx[coincide_ss[k]]):
                        if VisibilityRoadMap._point_in_polygon(mid_x, mid_y, all_expanded_obs[obs_i]):
                            blocked_by_mid[k] = True
                            break

                ss_blocked = blocked_by_int | blocked_by_mid
                if np.any(ss_blocked):
                    blocked_set = set()
                    for k in np.where(ss_blocked)[0]:
                        blocked_set.add((int(ss_i[k]), int(ss_j[k])))
                    for s_i in range(len(static_adj)):
                        road_map[N_d + s_i] = [j for j in road_map[N_d + s_i]
                                               if (N_d + s_i, j) not in blocked_set]

        # Build incremental pairs: only (i,j) where i<N_d OR j<N_d
        pi_all, pj_all = np.where(~np.eye(N_total, dtype=bool))
        mask = (pi_all < N_d) | (pj_all < N_d)
        pair_i = pi_all[mask]
        pair_j = pj_all[mask]

        tx = node_x[pair_i];  ty = node_y[pair_i]
        nx = node_x[pair_j];  ny = node_y[pair_j]

        tx_2d = tx[:, None];  ty_2d = ty[:, None]
        nx_2d = nx[:, None];  ny_2d = ny[:, None]
        p3x_2d = p3x[None, :]; p3y_2d = p3y[None, :]
        p4x_2d = p4x[None, :]; p4y_2d = p4y[None, :]

        eps = 1e-6

        coincide = ((np.abs(tx_2d - p3x_2d) < eps) & (np.abs(ty_2d - p3y_2d) < eps)) | \
                   ((np.abs(tx_2d - p4x_2d) < eps) & (np.abs(ty_2d - p4y_2d) < eps)) | \
                   ((np.abs(nx_2d - p3x_2d) < eps) & (np.abs(ny_2d - p3y_2d) < eps)) | \
                   ((np.abs(nx_2d - p4x_2d) < eps) & (np.abs(ny_2d - p4y_2d) < eps))

        o1 = (ny_2d - ty_2d) * (p3x_2d - nx_2d) - (nx_2d - tx_2d) * (p3y_2d - ny_2d)
        o2 = (ny_2d - ty_2d) * (p4x_2d - nx_2d) - (nx_2d - tx_2d) * (p4y_2d - ny_2d)
        o3 = (p4y_2d - p3y_2d) * (tx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ty_2d - p4y_2d)
        o4 = (p4y_2d - p3y_2d) * (nx_2d - p4x_2d) - (p4x_2d - p3x_2d) * (ny_2d - p4y_2d)

        s1 = np.sign(o1); s2 = np.sign(o2)
        s3 = np.sign(o3); s4 = np.sign(o4)

        general_intersect = (s1 != s2) & (s3 != s4)

        min_tx_nx = np.minimum(tx_2d, nx_2d); max_tx_nx = np.maximum(tx_2d, nx_2d)
        min_ty_ny = np.minimum(ty_2d, ny_2d); max_ty_ny = np.maximum(ty_2d, ny_2d)
        min_p3x_p4x = np.minimum(p3x_2d, p4x_2d); max_p3x_p4x = np.maximum(p3x_2d, p4x_2d)
        min_p3y_p4y = np.minimum(p3y_2d, p4y_2d); max_p3y_p4y = np.maximum(p3y_2d, p4y_2d)

        col1 = (s1 == 0) & (p3x_2d >= min_tx_nx) & (p3x_2d <= max_tx_nx) & \
                (p3y_2d >= min_ty_ny) & (p3y_2d <= max_ty_ny)
        col2 = (s2 == 0) & (p4x_2d >= min_tx_nx) & (p4x_2d <= max_tx_nx) & \
                (p4y_2d >= min_ty_ny) & (p4y_2d <= max_ty_ny)
        col3 = (s3 == 0) & (tx_2d >= min_p3x_p4x) & (tx_2d <= max_p3x_p4x) & \
                (ty_2d >= min_p3y_p4y) & (ty_2d <= max_p3y_p4y)
        col4 = (s4 == 0) & (nx_2d >= min_p3x_p4x) & (nx_2d <= max_p3x_p4x) & \
                (ny_2d >= min_p3y_p4y) & (ny_2d <= max_p3y_p4y)

        intersects = (~coincide) & (general_intersect | col1 | col2 | col3 | col4)
        pair_blocked_by_intersection = np.any(intersects, axis=1)
        has_coincidence = np.any(coincide, axis=1)
        needs_midpoint_check = has_coincidence & ~pair_blocked_by_intersection

        pair_blocked_by_midpoint = np.zeros(len(pair_i), dtype=bool)
        midpoint_indices = np.where(needs_midpoint_check)[0]
        if len(midpoint_indices) > 0:
            mid_x = (tx[midpoint_indices] + nx[midpoint_indices]) / 2.0
            mid_y = (ty[midpoint_indices] + ny[midpoint_indices]) / 2.0
            for k, pair_idx in enumerate(midpoint_indices):
                coincident_edges = coincide[pair_idx]
                obs_indices = np.unique(edge_obs_idx[coincident_edges])
                for obs_i in obs_indices:
                    if VisibilityRoadMap._point_in_polygon(mid_x[k], mid_y[k], all_expanded_obs[obs_i]):
                        pair_blocked_by_midpoint[pair_idx] = True
                        break

        pair_valid = ~pair_blocked_by_intersection & ~pair_blocked_by_midpoint
        for idx in np.where(pair_valid)[0]:
            road_map[pair_i[idx]].append(int(pair_j[idx]))

        return road_map

    def invalidate_cache(self):
        """Force a full static cache rebuild on the next planning call."""
        self._static_cache = None

    def _generate_road_map_info_vectorized(self, nodes, obstacles):
        """Fully vectorized road map generation using numpy broadcasting."""
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

        pair_blocked_by_midpoint = np.zeros(n_pairs, dtype=bool)
        midpoint_indices = np.where(needs_midpoint_check)[0]
        if len(midpoint_indices) > 0:
            mid_x = (tx[midpoint_indices] + nx[midpoint_indices]) / 2.0
            mid_y = (ty[midpoint_indices] + ny[midpoint_indices]) / 2.0
            for k, pair_idx in enumerate(midpoint_indices):
                coincident_edges = coincide[pair_idx]
                obs_indices = np.unique(edge_obs_idx[coincident_edges])
                for obs_i in obs_indices:
                    if VisibilityRoadMap._point_in_polygon(mid_x[k], mid_y[k], obstacles[obs_i]):
                        pair_blocked_by_midpoint[pair_idx] = True
                        break

        pair_valid = ~pair_blocked_by_intersection & ~pair_blocked_by_midpoint
        road_map_info_list = [[] for _ in range(n_nodes)]
        for idx in np.where(pair_valid)[0]:
            road_map_info_list[pair_i[idx]].append(int(pair_j[idx]))

        return road_map_info_list

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
