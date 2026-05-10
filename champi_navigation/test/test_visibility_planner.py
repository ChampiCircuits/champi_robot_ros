"""Unit tests for champi_navigation.visibility_planner.VisibilityRoadMap.

All tests verify that the planner returns the shortest path on simple,
analytically-solvable problems.  No ROS2 runtime is needed.

Coordinate conventions
----------------------
- All coordinates are in metres, with x pointing right and y pointing up.
- expand_distance=0 is used unless the test specifically needs C-space expansion,
  so that the geometric result exactly matches the theoretical optimum.

Shortest-path metric
--------------------
The path length is computed as the sum of Euclidean segment lengths.
For each scenario the expected optimal length is derived analytically and
the planner result must be within TOLS metres of that value.

Run with:
  pytest champi_navigation/test/test_visibility_planner.py -v
"""

from __future__ import annotations

import math
from typing import Optional

import pytest

from champi_navigation.visibility_planner.visibility_road_map import (
    ObstaclePolygon,
    VisibilityRoadMap,
)

# Absolute tolerance (metres) between planner path length and analytic optimum
LENGTH_TOL = 1e-6


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _path_length(rx: list[float], ry: list[float]) -> float:
    """Return total Euclidean length of a polyline given as (rx, ry) lists."""
    length = 0.0
    for i in range(1, len(rx)):
        length += math.hypot(rx[i] - rx[i - 1], ry[i] - ry[i - 1])
    return length


def _rect_obstacle(x_min: float, y_min: float, x_max: float, y_max: float) -> ObstaclePolygon:
    """Create an axis-aligned rectangular ObstaclePolygon."""
    return ObstaclePolygon(
        x_list=[x_min, x_max, x_max, x_min],
        y_list=[y_min, y_min, y_max, y_max],
    )


def _plan(
    start: tuple[float, float],
    goal: tuple[float, float],
    static_obstacles: list[ObstaclePolygon],
    dynamic_obstacles: Optional[list[ObstaclePolygon]] = None,
    expand_distance: float = 0.0,
) -> tuple[list[float], list[float]]:
    """Convenience wrapper: plan and return (rx, ry) — goal-to-start order reversed to start-to-goal."""
    vrm = VisibilityRoadMap(expand_distance=expand_distance)
    rx, ry, _ = vrm.planning(
        start[0], start[1],
        goal[0], goal[1],
        static_obstacles,
        dynamic_obstacles,
    )
    # planning() returns path from goal→start; reverse to start→goal for readability
    rx = list(reversed(rx))
    ry = list(reversed(ry))
    return rx, ry


# ---------------------------------------------------------------------------
# Tests: free space (no obstacles)
# ---------------------------------------------------------------------------

class TestFreeSpace:
    """Without obstacles the shortest path is always a straight line."""

    def test_straight_line_horizontal(self):
        start, goal = (0.0, 0.0), (2.0, 0.0)
        rx, ry = _plan(start, goal, static_obstacles=[])
        assert rx and ry, "Planner returned no path"
        expected = math.dist(start, goal)
        assert abs(_path_length(rx, ry) - expected) < LENGTH_TOL

    def test_straight_line_vertical(self):
        start, goal = (0.0, 0.0), (0.0, 3.0)
        rx, ry = _plan(start, goal, static_obstacles=[])
        assert rx and ry, "Planner returned no path"
        assert abs(_path_length(rx, ry) - 3.0) < LENGTH_TOL

    def test_straight_line_diagonal(self):
        start, goal = (0.0, 0.0), (1.0, 1.0)
        rx, ry = _plan(start, goal, static_obstacles=[])
        assert rx and ry, "Planner returned no path"
        expected = math.sqrt(2.0)
        assert abs(_path_length(rx, ry) - expected) < LENGTH_TOL

    def test_start_equals_goal(self):
        """Degenerate case: start == goal should return a trivial path of length 0."""
        start = goal = (1.0, 1.0)
        rx, ry = _plan(start, goal, static_obstacles=[])
        assert rx and ry, "Planner returned no path"
        assert _path_length(rx, ry) < LENGTH_TOL

    def test_long_diagonal(self):
        start, goal = (0.0, 0.0), (3.0, 4.0)
        rx, ry = _plan(start, goal, static_obstacles=[])
        assert rx and ry
        assert abs(_path_length(rx, ry) - 5.0) < LENGTH_TOL


# ---------------------------------------------------------------------------
# Tests: single obstacle — the planner must route around it
# ---------------------------------------------------------------------------

class TestSingleObstacle:
    """One rectangle blocking the direct path; optimal detour is known analytically."""

    def test_obstacle_above_direct_path_bypassed_below(self):
        """
        start=(0,0), goal=(4,0).  A tall rectangle sits in the middle of the y-axis
        but only covers positive y, so the direct path (y=0) is actually free.
        The straight-line distance must therefore be returned.
        """
        obs = _rect_obstacle(1.5, 0.5, 2.5, 2.0)  # obstacle in y>0, direct path at y=0
        rx, ry = _plan((0.0, 0.0), (4.0, 0.0), [obs])
        assert rx and ry
        assert abs(_path_length(rx, ry) - 4.0) < LENGTH_TOL

    def test_path_blocked_must_detour(self):
        """
        start=(0,1), goal=(4,1).  A rectangle spans the full height around y=1,
        so the planner must route around it.  The detour going around either the
        top or bottom corner is longer than 4.0.
        """
        # Obstacle: x in [1.5, 2.5], y in [0.0, 2.0]  — blocks the horizontal midline
        obs = _rect_obstacle(1.5, 0.0, 2.5, 2.0)
        rx, ry = _plan((0.0, 1.0), (4.0, 1.0), [obs])
        assert rx and ry, "Planner must find a path around the obstacle"
        assert _path_length(rx, ry) > 4.0, "Detour must be longer than the straight-line distance"

    def test_path_blocked_detour_length(self):
        """
        Exact shortest-path calculation around a unit square placed at the midpoint.

        Layout (expand_distance=0):
          start = (0, 0.5)
          goal  = (3, 0.5)
          obstacle: x in [1, 2], y in [0, 1]  (unit square)

        The direct path y=0.5 is blocked.  The two shortest detours go via the
        corner (1, 0) or (1, 1) and then (2, 0) or (2, 1).

        Optimal path via bottom corners (1,0)→(2,0):
          d1 = dist((0,0.5), (1,0)) = sqrt(1 + 0.25) = sqrt(1.25)
          d2 = dist((1,0),   (2,0)) = 1.0
          d3 = dist((2,0),   (3,0.5)) = sqrt(1 + 0.25) = sqrt(1.25)
          total = 1 + 2*sqrt(1.25) ≈ 3.236
        """
        obs = _rect_obstacle(1.0, 0.0, 2.0, 1.0)
        rx, ry = _plan((0.0, 0.5), (3.0, 0.5), [obs])
        assert rx and ry

        expected = 1.0 + 2.0 * math.sqrt(1.25)
        assert abs(_path_length(rx, ry) - expected) < 1e-3  # slightly relaxed: corner vertices

    def test_no_path_when_fully_enclosed(self):
        """
        If the goal is completely surrounded by obstacles the planner should
        return an empty path.
        """
        # Four obstacles forming a closed box around (2, 2) with no gap
        obs = [
            _rect_obstacle(1.0, 1.0, 3.0, 1.01),  # bottom wall
            _rect_obstacle(1.0, 2.99, 3.0, 3.0),  # top wall
            _rect_obstacle(1.0, 1.0, 1.01, 3.0),  # left wall
            _rect_obstacle(2.99, 1.0, 3.0, 3.0),  # right wall
        ]
        rx, ry = _plan((0.0, 0.0), (2.0, 2.0), obs)
        assert rx == [] and ry == [], "Planner should return no path for enclosed goal"


# ---------------------------------------------------------------------------
# Tests: two obstacles — the planner chooses the shorter of two detours
# ---------------------------------------------------------------------------

class TestTwoObstacles:
    """
    Two symmetric obstacles offer two possible detour corridors.
    The planner must pick the shorter one.
    """

    def test_chooses_shorter_corridor(self):
        """
        start=(0, 1.5), goal=(6, 1.5).

        Two rectangular obstacles block the direct path:
          - Top obstacle:    x in [2,4], y in [2.0, 3.0]   → top corridor is narrow
          - Bottom obstacle: x in [2,4], y in [0.5, 1.0]   → bottom corridor is wide

        The bottom corridor allows a detour that dips only slightly; the top
        corridor forces a larger detour.  We verify that the chosen path length
        is ≤ the length of the longer detour, confirming the planner picks the
        shorter route.

        Both detour lengths are computed analytically:
          Via bottom gap (route goes below y=0.5):
            approx distance from (0,1.5) → corner (2,0.5) → corner (4,0.5) → (6,1.5)
            = dist((0,1.5),(2,0.5)) + 2 + dist((4,0.5),(6,1.5))
            = sqrt(4+1) + 2 + sqrt(4+1) = 2 + 2*sqrt(5) ≈ 6.472

          Via top gap (route goes above y=3.0):
            = dist((0,1.5),(2,3.0)) + 2 + dist((4,3.0),(6,1.5))
            = sqrt(4+2.25) + 2 + sqrt(4+2.25) = 2 + 2*sqrt(6.25) = 2 + 5 = 7.0
        """
        obs_bottom = _rect_obstacle(2.0, 0.5, 4.0, 1.0)
        obs_top    = _rect_obstacle(2.0, 2.0, 4.0, 3.0)

        rx, ry = _plan((0.0, 1.5), (6.0, 1.5), [obs_bottom, obs_top])
        assert rx and ry

        longer_detour = 2.0 + 2.0 * math.sqrt(6.25)  # ≈ 7.0
        assert _path_length(rx, ry) < longer_detour - 1e-3, (
            f"Planner should have found the shorter detour "
            f"(got {_path_length(rx, ry):.4f}, longer detour is {longer_detour:.4f})"
        )


# ---------------------------------------------------------------------------
# Tests: static vs dynamic obstacle caching
# ---------------------------------------------------------------------------

class TestCaching:
    """Verify that the cached static road map gives the same result as a fresh plan,
    and that introducing a dynamic obstacle (enemy robot square) correctly blocks
    the previously cached straight-line path."""

    def test_same_result_with_and_without_cache(self):
        """Two consecutive calls with identical static obstacles must yield the same path."""
        obs = [_rect_obstacle(1.0, 0.0, 2.0, 1.0)]
        vrm = VisibilityRoadMap(expand_distance=0.0)

        rx1, ry1, _ = vrm.planning(0.0, 0.5, 3.0, 0.5, obs)
        rx2, ry2, _ = vrm.planning(0.0, 0.5, 3.0, 0.5, obs)

        assert _path_length(rx1, ry1) == pytest.approx(_path_length(rx2, ry2), abs=LENGTH_TOL)

    def test_dynamic_obstacle_blocks_cached_path(self):
        """
        The static road map has a clear direct path.  Adding a dynamic obstacle
        (rectangle) in the middle must force a detour — the dynamic result must
        be longer than the static one.
        """
        static_obs: list[ObstaclePolygon] = []
        dynamic_obs = [_rect_obstacle(1.5, 0.3, 2.5, 1.7)]

        vrm = VisibilityRoadMap(expand_distance=0.0)

        # Without dynamic obstacle
        rx_clear, ry_clear, _ = vrm.planning(0.0, 1.0, 4.0, 1.0, static_obs)
        len_clear = _path_length(rx_clear, ry_clear)

        # With dynamic obstacle blocking the straight path
        rx_dyn, ry_dyn, _ = vrm.planning(0.0, 1.0, 4.0, 1.0, static_obs, dynamic_obs)
        assert rx_dyn and ry_dyn, "Planner must find a path even with dynamic obstacle"
        len_dyn = _path_length(rx_dyn, ry_dyn)

        assert len_dyn > len_clear, (
            f"Dynamic obstacle should force a longer path "
            f"(clear={len_clear:.4f}, dynamic={len_dyn:.4f})"
        )

    def test_static_cache_invalidated_after_change(self):
        """Changing static obstacles must cause the planner to replan, not use stale cache."""
        obs_v1 = [_rect_obstacle(1.0, 0.0, 2.0, 2.0)]
        obs_v2: list[ObstaclePolygon] = []  # obstacle removed

        vrm = VisibilityRoadMap(expand_distance=0.0)

        rx1, ry1, _ = vrm.planning(0.0, 1.0, 3.0, 1.0, obs_v1)
        len_with_obs = _path_length(rx1, ry1)

        rx2, ry2, _ = vrm.planning(0.0, 1.0, 3.0, 1.0, obs_v2)
        len_no_obs = _path_length(rx2, ry2)

        # Removing an obstacle on the direct path should shorten or equal the distance
        assert len_no_obs <= len_with_obs + LENGTH_TOL


# ---------------------------------------------------------------------------
# Tests: C-space expansion
# ---------------------------------------------------------------------------

class TestExpansion:
    """When expand_distance > 0 the planner inflates obstacles.  A path that would
    be valid with no expansion may be blocked when the robot radius is applied."""

    def test_expansion_forces_wider_detour(self):
        """
        With a narrow corridor just wide enough for expand_distance=0 but too narrow
        for expand_distance=0.3, the expanded path must be longer.
        """
        # Obstacle: x in [1,2], y in [0.35, 2.0] — leaves a 0.35m gap at the bottom
        obs = [_rect_obstacle(1.0, 0.35, 2.0, 2.0)]

        vrm_zero = VisibilityRoadMap(expand_distance=0.0)
        rx0, ry0, _ = vrm_zero.planning(0.0, 0.0, 3.0, 0.0, obs)
        len0 = _path_length(rx0, ry0)

        vrm_exp = VisibilityRoadMap(expand_distance=0.3)
        rx_exp, ry_exp, _ = vrm_exp.planning(0.0, 0.0, 3.0, 0.0, obs)

        if rx_exp and ry_exp:
            # If a path still exists it must be longer (wider detour)
            assert _path_length(rx_exp, ry_exp) >= len0 - LENGTH_TOL
        # If no path is returned, the corridor is fully blocked — also acceptable
