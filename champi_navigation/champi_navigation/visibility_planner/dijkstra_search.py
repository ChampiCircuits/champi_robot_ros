"""

Graph based Dijkstra search for road map planning.

"""

import math


class DijkstraSearch:

    class Node:
        def __init__(self, x, y, cost=None, parent=None):
            self.x = x
            self.y = y
            self.cost = cost if cost is not None else float('inf')
            self.parent = parent

    def __init__(self):
        pass

    def search(self, start_x, start_y, goal_x, goal_y,
               node_x_list, node_y_list, road_map_info):
        """
        Dijkstra search on a road map graph.

        Args:
            start_x, start_y: start position
            goal_x, goal_y: goal position
            node_x_list: list of x positions of all nodes
            node_y_list: list of y positions of all nodes
            road_map_info: adjacency list (list of lists of connected node indices)

        Returns:
            rx, ry: lists of x and y positions of the path (from goal to start)
        """
        start_node_idx = self._find_nearest_node(start_x, start_y, node_x_list, node_y_list)
        goal_node_idx = self._find_nearest_node(goal_x, goal_y, node_x_list, node_y_list)

        n_nodes = len(node_x_list)
        cost = [float('inf')] * n_nodes
        parent = [-1] * n_nodes
        visited = [False] * n_nodes

        cost[start_node_idx] = 0.0

        while True:
            # Find the unvisited node with the smallest cost
            min_cost = float('inf')
            min_idx = -1
            for i in range(n_nodes):
                if not visited[i] and cost[i] < min_cost:
                    min_cost = cost[i]
                    min_idx = i

            if min_idx == -1:
                # No path found
                return [], []

            if min_idx == goal_node_idx:
                break

            visited[min_idx] = True

            # Explore neighbors
            for neighbor_idx in road_map_info[min_idx]:
                if visited[neighbor_idx]:
                    continue
                dx = node_x_list[min_idx] - node_x_list[neighbor_idx]
                dy = node_y_list[min_idx] - node_y_list[neighbor_idx]
                edge_cost = math.hypot(dx, dy)
                new_cost = cost[min_idx] + edge_cost
                if new_cost < cost[neighbor_idx]:
                    cost[neighbor_idx] = new_cost
                    parent[neighbor_idx] = min_idx

        # Reconstruct path
        rx, ry = [], []
        idx = goal_node_idx
        while idx != -1:
            rx.append(node_x_list[idx])
            ry.append(node_y_list[idx])
            idx = parent[idx]

        return rx, ry

    @staticmethod
    def _find_nearest_node(x, y, node_x_list, node_y_list):
        min_dist = float('inf')
        min_idx = 0
        for i, (nx, ny) in enumerate(zip(node_x_list, node_y_list)):
            dist = math.hypot(x - nx, y - ny)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

