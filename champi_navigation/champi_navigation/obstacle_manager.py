"""
ObstacleManager — maintains environment state and performs geometric collision checks.

This module is intentionally free of ROS2 dependencies so it can be unit-tested
with plain pytest without a running ROS stack.
"""

from __future__ import annotations

import math
from typing import Optional

import yaml

from champi_navigation.visibility_planner.visibility_road_map import (
    ObstaclePolygon,
    VisibilityRoadMap,
)


class ObstacleManager:
    """Maintains the state of the planning environment.

    Responsibilities:
    - Load static geometry (table borders, zones, elements) from a YAML config file.
    - Track mutable state: which zones/elements are currently obstacles, enemy position.
    - Build :class:`ObstaclePolygon` lists for the path planner and collision checker.
    - Answer point-in-forbidden-area queries and compute nearest exit points.

    This class is ROS2-free. Callers (e.g. the planner node) are responsible for
    resolving package paths, publishing messages, and calling the path planner.

    Dependency injection:
    - ``config_path``: fully-resolved path to the world-state YAML file.
      The node resolves this with ``get_package_share_directory``; tests pass a fixture path.
    - ``forbidden_area_checker``: optional custom :class:`VisibilityRoadMap` instance.
      When omitted one is created from *robot_radius* and *forbidden_area_margin*.
    """

    # Physical dimensions of a nut_box element (metres, full width/height)
    _NUT_BOX_W: float = 0.15
    _NUT_BOX_H: float = 0.20

    # How far outside an expanded polygon boundary the exit point is placed (metres)
    _EXIT_OFFSET: float = 0.03

    def __init__(
        self,
        config_path: str,
        table_width: float,
        table_height: float,
        robot_radius: float,
        enemy_robot_radius: float,
        forbidden_area_margin: float,
        forbidden_area_checker: Optional[VisibilityRoadMap] = None,
    ) -> None:
        self._table_width = table_width
        self._table_height = table_height
        self._robot_radius = robot_radius
        self._enemy_robot_radius = enemy_robot_radius

        # Collision checker uses a slightly smaller expansion than the path planner
        # to avoid false triggers when the robot is just at the C-space boundary.
        if forbidden_area_checker is not None:
            self._checker = forbidden_area_checker
        else:
            expand = max(0.0, robot_radius - forbidden_area_margin)
            self._checker = VisibilityRoadMap(expand_distance=expand)

        self._zones, self._elements = self._load_config(config_path)
        self._elements = [] # TODO FIX: not used. and was crashing state_machine when element was not defined in world_state.yaml 
        self._element_ids: set[str] = {e["id"] for e in self._elements}

        # placement_zones start free (not an obstacle); all other zone types start occupied.
        # secure_zones are never added as obstacles but we still track their state in case
        # the brain sends updates for them.
        self._zone_occupied: dict[str, bool] = {
            z["id"]: z.get("type") != "placement_zone"
            for z in self._zones
        }

        # IDs of elements that have been picked up (removed from obstacle list)
        self._elements_disabled: set[str] = set()

        # Enemy robot position; None when unknown
        self._enemy_pose: Optional[tuple[float, float]] = None

        # Pre-build immutable border obstacles (table edges never change)
        self._border_obstacles: list[ObstaclePolygon] = _build_border_obstacles(
            self._table_width, self._table_height
        )

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    @staticmethod
    def _load_config(config_path: str) -> tuple[list[dict], list[dict]]:
        """Parse the world-state YAML and return (zones, elements)."""
        with open(config_path, "r") as fh:
            world_state = yaml.safe_load(fh)
        return world_state.get("zones", []), world_state.get("elements", [])

    # ------------------------------------------------------------------
    # State updates
    # ------------------------------------------------------------------

    def update_states(self, states: dict[str, bool]) -> bool:
        """Apply a full obstacle-state snapshot from the brain.

        Args:
            states: mapping of *entity_id* → *is_obstacle*.

        Returns:
            ``True`` if at least one state actually changed.  The caller
            should invalidate the path-planner cache when this returns ``True``.
        """
        changed = False
        for entity_id, is_obstacle in states.items():
            if entity_id in self._zone_occupied:
                if self._zone_occupied[entity_id] != is_obstacle:
                    self._zone_occupied[entity_id] = is_obstacle
                    changed = True
            elif entity_id in self._element_ids:
                was_disabled = entity_id in self._elements_disabled
                if not is_obstacle and not was_disabled:
                    # Element taken by robot → remove from obstacles
                    self._elements_disabled.add(entity_id)
                    changed = True
                elif is_obstacle and was_disabled:
                    # Element restored (e.g. placed back)
                    self._elements_disabled.discard(entity_id)
                    changed = True
        return changed

    def set_enemy_pose(self, x: float, y: float) -> None:
        """Update the last-known enemy robot position."""
        self._enemy_pose = (x, y)

    def clear_enemy_pose(self) -> None:
        """Remove the enemy robot from the obstacle lists."""
        self._enemy_pose = None

    # ------------------------------------------------------------------
    # Obstacle builders (private helpers)
    # ------------------------------------------------------------------

    def _build_zone_obstacles(self) -> list[ObstaclePolygon]:
        """Axis-aligned rectangles for all currently-occupied, non-secure zones."""
        obstacles: list[ObstaclePolygon] = []
        for zone in self._zones:
            if zone.get("type") == "secure_zone":
                continue
            if not self._zone_occupied.get(zone["id"], False):
                continue
            x, y = float(zone["x"]), float(zone["y"])
            w, h = float(zone["width"]), float(zone["height"])
            obstacles.append(
                ObstaclePolygon(
                    [x, x + w, x + w, x],
                    [y, y, y + h, y + h],
                )
            )
        return obstacles

    def _build_element_obstacles(self) -> list[ObstaclePolygon]:
        """Rotated bounding rectangles for all active nut_box elements."""
        hw = self._NUT_BOX_W / 2.0
        hh = self._NUT_BOX_H / 2.0
        obstacles: list[ObstaclePolygon] = []
        for elem in self._elements:
            if elem.get("type") != "nut_box":
                continue
            if elem["id"] in self._elements_disabled:
                continue
            cx, cy = float(elem["x"]), float(elem["y"])
            # +90° converts from the element's local frame to the world frame
            theta = math.radians(float(elem.get("theta_deg", 0.0)) + 90.0)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            xs = [cx + cos_t * lx - sin_t * ly for lx, ly in corners]
            ys = [cy + sin_t * lx + cos_t * ly for lx, ly in corners]
            obstacles.append(ObstaclePolygon(xs, ys))
        return obstacles

    def _build_enemy_obstacle(self, x: float, y: float) -> ObstaclePolygon:
        """Square obstacle of side ``2 * enemy_robot_radius`` centred on *x, y*."""
        r = self._enemy_robot_radius
        return ObstaclePolygon(
            [x - r, x + r, x + r, x - r],
            [y - r, y - r, y + r, y + r],
        )

    # ------------------------------------------------------------------
    # Table-area filter
    # ------------------------------------------------------------------

    def _filter_outside_table(
        self, obstacles: list[ObstaclePolygon]
    ) -> list[ObstaclePolygon]:
        """Drop obstacles whose bounding box does not overlap the table + margin.

        The margin equals *robot_radius* so C-space expansion near the borders
        is never accidentally discarded.
        """
        margin = self._robot_radius
        result: list[ObstaclePolygon] = []
        for obs in obstacles:
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            if (
                max(xs) >= -margin
                and min(xs) <= self._table_width + margin
                and max(ys) >= -margin
                and min(ys) <= self._table_height + margin
            ):
                result.append(obs)
        return result

    # ------------------------------------------------------------------
    # Public obstacle accessors
    # ------------------------------------------------------------------

    def get_static_obstacles(self) -> list[ObstaclePolygon]:
        """Borders + occupied zones + active elements, filtered to the table area.

        Use this for path planning (cached by the :class:`VisibilityRoadMap`).
        """
        obstacles: list[ObstaclePolygon] = list(self._border_obstacles)
        obstacles.extend(self._build_zone_obstacles())
        obstacles.extend(self._build_element_obstacles())
        return self._filter_outside_table(obstacles)

    def get_dynamic_obstacles(self) -> list[ObstaclePolygon]:
        """Enemy robot obstacle (filtered to the table area), or empty list.

        Use this for path planning; recomputed every planning call.
        """
        if self._enemy_pose is not None:
            return self._filter_outside_table(
                [self._build_enemy_obstacle(*self._enemy_pose)]
            )
        return []
    
    def get_borders_obstacles(self) -> list[ObstaclePolygon]:
        return self._border_obstacles

    def get_all_obstacles(self) -> list[ObstaclePolygon]:
        """All obstacles without table-area filtering.

        Used for collision checks (forbidden-area detection) where accuracy near
        the table edge matters more than performance.
        """
        obstacles: list[ObstaclePolygon] = list(self._border_obstacles)
        obstacles.extend(self._build_zone_obstacles())
        obstacles.extend(self._build_element_obstacles())
        if self._enemy_pose is not None:
            obstacles.append(self._build_enemy_obstacle(*self._enemy_pose))
        return obstacles

    # ------------------------------------------------------------------
    # Geometric collision checks
    # ------------------------------------------------------------------

    def is_point_in_forbidden_area(self, x: float, y: float) -> bool:
        """Return ``True`` if *(x, y)* lies inside any expanded (C-space) obstacle."""
        expanded = self._checker.build_expanded_obstacles(self.get_all_obstacles())
        return any(
            VisibilityRoadMap._point_in_polygon(x, y, obs) for obs in expanded
        )

    def find_nearest_exit_point(
        self, x: float, y: float
    ) -> Optional[tuple[float, float]]:
        """Return the closest point outside all expanded obstacles.

        Projects *(x, y)* onto each expanded polygon's boundary and nudges it
        outward by :attr:`_EXIT_OFFSET` metres.  Returns ``None`` if the point
        is not inside any expanded obstacle.

        Args:
            x: X coordinate of the point to escape from.
            y: Y coordinate of the point to escape from.

        Returns:
            ``(exit_x, exit_y)`` or ``None``.
        """
        expanded = self._checker.build_expanded_obstacles(self.get_all_obstacles())
        best_point: Optional[tuple[float, float]] = None
        best_dist = float("inf")

        for obs in expanded:
            if not VisibilityRoadMap._point_in_polygon(x, y, obs):
                continue

            n_verts = len(obs.x_list) - 1
            cx_poly = sum(obs.x_list[:n_verts]) / n_verts
            cy_poly = sum(obs.y_list[:n_verts]) / n_verts

            for i in range(n_verts):
                ax, ay = obs.x_list[i], obs.y_list[i]
                bx, by = obs.x_list[i + 1], obs.y_list[i + 1]
                dx, dy = bx - ax, by - ay
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-9:
                    continue
                t = max(
                    0.0,
                    min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq),
                )
                proj_x = ax + t * dx
                proj_y = ay + t * dy

                # Outward normal (perpendicular to edge, pointing away from the
                # polygon interior)
                nx_edge, ny_edge = -dy, dx
                norm = math.hypot(nx_edge, ny_edge)
                if norm < 1e-9:
                    continue
                nx_edge /= norm
                ny_edge /= norm
                if nx_edge * (proj_x - cx_poly) + ny_edge * (proj_y - cy_poly) < 0:
                    nx_edge, ny_edge = -nx_edge, -ny_edge

                exit_x = proj_x + nx_edge * self._EXIT_OFFSET
                exit_y = proj_y + ny_edge * self._EXIT_OFFSET
                dist = math.hypot(x - exit_x, y - exit_y)
                if dist < best_dist:
                    best_dist = dist
                    best_point = (exit_x, exit_y)

        return best_point


# ------------------------------------------------------------------
# Module-level helpers (used internally; exposed for testing)
# ------------------------------------------------------------------

def _build_border_obstacles(table_width: float, table_height: float) -> list[ObstaclePolygon]:
    """Four thin rectangular walls forming the table boundary.

    Each wall is 1 cm thick and sits just outside the playfield edge so that
    C-space expansion pushes the robot's safe region inward by *robot_radius*.
    """
    w, h, t = table_width, table_height, 0.01
    return [
        ObstaclePolygon([0.0, w,   w,   0.0], [0.0,  0.0,  -t,  -t]),   # bottom
        ObstaclePolygon([0.0, w,   w,   0.0], [h,    h,   h+t, h+t]),   # top
        ObstaclePolygon([0.0, 0.0, -t,  -t],  [0.0,  h,    h,  0.0]),   # left
        ObstaclePolygon([w,   w,  w+t, w+t],  [0.0,  h,    h,  0.0]),   # right
    ]
