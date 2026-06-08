"""Unit tests for champi_navigation.obstacle_manager.ObstacleManager.

No ROS2 infrastructure is needed — ObstacleManager is ROS2-free.
Run with:  pytest champi_navigation/test/test_obstacle_manager.py
"""

from __future__ import annotations

import math
import pathlib

import pytest

from champi_navigation.obstacle_manager import ObstacleManager, _build_border_obstacles
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "world_state_test.yaml"
TABLE_W = 3.0
TABLE_H = 2.0
ROBOT_R = 0.15
ENEMY_R = 0.20
MARGIN = 0.05  # forbidden_area_margin → expand_distance = ROBOT_R - MARGIN = 0.10


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def manager() -> ObstacleManager:
    return ObstacleManager(
        config_path=str(FIXTURE),
        table_width=TABLE_W,
        table_height=TABLE_H,
        robot_radius=ROBOT_R,
        enemy_robot_radius=ENEMY_R,
        forbidden_area_margin=MARGIN,
    )


# ---------------------------------------------------------------------------
# Border obstacles
# ---------------------------------------------------------------------------


class TestBorderObstacles:
    def test_four_borders_returned(self):
        borders = _build_border_obstacles(TABLE_W, TABLE_H)
        assert len(borders) == 4

    def test_borders_cover_all_four_sides(self):
        """Each border polygon should bracket one edge of the table."""
        borders = _build_border_obstacles(TABLE_W, TABLE_H)
        # Collect bounding boxes
        bbs = [
            (min(b.x_list), max(b.x_list), min(b.y_list), max(b.y_list))
            for b in borders
        ]
        # Bottom: y straddles 0
        assert any(ymin < 0 and ymax >= 0 and xmax >= TABLE_W - 0.01 for _, xmax, ymin, ymax in bbs)
        # Top: y straddles TABLE_H
        assert any(ymin <= TABLE_H and ymax > TABLE_H and xmax >= TABLE_W - 0.01 for _, xmax, ymin, ymax in bbs)
        # Left: x straddles 0
        assert any(xmin < 0 and xmax >= 0 and ymax >= TABLE_H - 0.01 for xmin, xmax, _, ymax in bbs)
        # Right: x straddles TABLE_W
        assert any(xmin <= TABLE_W and xmax > TABLE_W and ymax >= TABLE_H - 0.01 for xmin, xmax, _, ymax in bbs)

    def test_border_obstacles_cached_in_manager(self, manager: ObstacleManager):
        assert len(manager._border_obstacles) == 4


# ---------------------------------------------------------------------------
# Zone initial state
# ---------------------------------------------------------------------------


class TestZoneInitialState:
    def test_forbidden_zone_starts_occupied(self, manager: ObstacleManager):
        assert manager._zone_occupied["forbidden_zone_1"] is True

    def test_placement_zone_starts_free(self, manager: ObstacleManager):
        assert manager._zone_occupied["placement_zone_1"] is False

    def test_secure_zone_tracked_but_starts_occupied(self, manager: ObstacleManager):
        # secure_zone is tracked but never added to obstacle lists
        assert "secure_zone_1" in manager._zone_occupied


# ---------------------------------------------------------------------------
# Zone obstacle builder
# ---------------------------------------------------------------------------


class TestBuildZoneObstacles:
    def test_occupied_forbidden_zone_included(self, manager: ObstacleManager):
        obstacles = manager._build_zone_obstacles()
        # forbidden_zone_1 is occupied; placement_zone_1 is free; secure_zone excluded
        assert len(obstacles) == 1

    def test_secure_zone_never_included(self, manager: ObstacleManager):
        # Mark secure_zone_1 as occupied; it must still be absent from obstacle list
        manager._zone_occupied["secure_zone_1"] = True
        obstacles = manager._build_zone_obstacles()
        for obs in obstacles:
            # secure_zone_1 is at x=2.0, y=1.5; forbidden_zone_1 at x=1.0, y=0.5
            # None should overlap the secure zone position
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            assert not (min(xs) >= 1.9 and min(ys) >= 1.4), "secure_zone should not appear"

    def test_free_placement_zone_excluded(self, manager: ObstacleManager):
        obstacles = manager._build_zone_obstacles()
        # placement_zone_1 is at x=0.1, y=0.1 — must not appear
        for obs in obstacles:
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            assert not (max(xs) <= 0.32 and max(ys) <= 0.32), (
                "placement_zone_1 should not appear when free"
            )

    def test_zone_polygon_dimensions(self, manager: ObstacleManager):
        """forbidden_zone_1 is 0.5 x 0.5 at (1.0, 0.5)."""
        obstacles = manager._build_zone_obstacles()
        assert len(obstacles) == 1
        obs = obstacles[0]
        xs = sorted(obs.x_list[:-1])
        ys = sorted(obs.y_list[:-1])
        assert abs(min(xs) - 1.0) < 1e-9
        assert abs(max(xs) - 1.5) < 1e-9
        assert abs(min(ys) - 0.5) < 1e-9
        assert abs(max(ys) - 1.0) < 1e-9


# ---------------------------------------------------------------------------
# update_states
# ---------------------------------------------------------------------------


class TestUpdateStates:
    def test_no_change_returns_false(self, manager: ObstacleManager):
        # forbidden_zone_1 is already True
        changed = manager.update_states({"forbidden_zone_1": True})
        assert changed is False

    def test_zone_change_returns_true(self, manager: ObstacleManager):
        changed = manager.update_states({"placement_zone_1": True})
        assert changed is True

    def test_zone_state_updated(self, manager: ObstacleManager):
        manager.update_states({"placement_zone_1": True})
        assert manager._zone_occupied["placement_zone_1"] is True

    def test_now_occupied_placement_zone_appears_in_obstacles(self, manager: ObstacleManager):
        manager.update_states({"placement_zone_1": True})
        obstacles = manager._build_zone_obstacles()
        # forbidden_zone_1 + placement_zone_1 = 2
        assert len(obstacles) == 2

    def test_free_forbidden_zone_disappears(self, manager: ObstacleManager):
        manager.update_states({"forbidden_zone_1": False})
        obstacles = manager._build_zone_obstacles()
        assert len(obstacles) == 0

    def test_element_disabled_returns_true(self, manager: ObstacleManager):
        changed = manager.update_states({"box_1": False})
        assert changed is True
        assert "box_1" in manager._elements_disabled

    def test_element_disabled_no_longer_built(self, manager: ObstacleManager):
        manager.update_states({"box_1": False})
        obstacles = manager._build_element_obstacles()
        # Only box_2 remains
        assert len(obstacles) == 1

    def test_element_restored_returns_true(self, manager: ObstacleManager):
        manager.update_states({"box_1": False})
        changed = manager.update_states({"box_1": True})
        assert changed is True
        assert "box_1" not in manager._elements_disabled

    def test_element_restored_reappears(self, manager: ObstacleManager):
        manager.update_states({"box_1": False})
        manager.update_states({"box_1": True})
        obstacles = manager._build_element_obstacles()
        assert len(obstacles) == 2

    def test_unknown_entity_ignored_no_change(self, manager: ObstacleManager):
        changed = manager.update_states({"nonexistent_id": True})
        assert changed is False

    def test_multiple_updates_in_one_call(self, manager: ObstacleManager):
        changed = manager.update_states({
            "placement_zone_1": True,
            "box_1": False,
        })
        assert changed is True
        assert manager._zone_occupied["placement_zone_1"] is True
        assert "box_1" in manager._elements_disabled


# ---------------------------------------------------------------------------
# Element obstacle builder
# ---------------------------------------------------------------------------


class TestBuildElementObstacles:
    def test_two_elements_initially(self, manager: ObstacleManager):
        obstacles = manager._build_element_obstacles()
        assert len(obstacles) == 2

    def test_element_obstacle_is_a_rectangle(self, manager: ObstacleManager):
        """Each element obstacle should be a 4-vertex (closed) polygon."""
        for obs in manager._build_element_obstacles():
            # x_list and y_list are closed (first == last)
            assert obs.x_list[0] == obs.x_list[-1]
            assert obs.y_list[0] == obs.y_list[-1]
            assert len(obs.x_list) - 1 == 4  # 4 distinct vertices

    def test_element_centred_on_element_position(self, manager: ObstacleManager):
        """The bounding-box centre of each element obstacle should equal elem (x, y)."""
        obstacles = manager._build_element_obstacles()
        expected_centres = [(1.5, 1.0), (2.0, 0.5)]
        for obs in obstacles:
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
            # One of the expected centres should match
            assert any(
                abs(cx - ex) < 1e-6 and abs(cy - ey) < 1e-6
                for ex, ey in expected_centres
            ), f"No expected centre matches ({cx:.4f}, {cy:.4f})"

    def test_element_obstacle_dimensions(self, manager: ObstacleManager):
        """The axis-aligned bounding box of a rotated box should enclose the box."""
        hw = ObstacleManager._NUT_BOX_W / 2.0
        hh = ObstacleManager._NUT_BOX_H / 2.0
        # box_1 at (1.5, 1.0), theta_deg=0 → rotation = 90° → box is transposed
        # After +90° rotation: local x maps to world y, local y maps to world -x
        # So bounding box: width = NUT_BOX_H, height = NUT_BOX_W
        obstacles = manager._build_element_obstacles()
        box1_obs = None
        for obs in obstacles:
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
            if abs(cx - 1.5) < 1e-6 and abs(cy - 1.0) < 1e-6:
                box1_obs = obs
                break
        assert box1_obs is not None
        xs = box1_obs.x_list[:-1]
        ys = box1_obs.y_list[:-1]
        bb_w = max(xs) - min(xs)
        bb_h = max(ys) - min(ys)
        # theta_deg=0 + 90° → 90° rotation: NUT_BOX_W along Y, NUT_BOX_H along X
        assert abs(bb_w - ObstacleManager._NUT_BOX_H) < 1e-6
        assert abs(bb_h - ObstacleManager._NUT_BOX_W) < 1e-6


# ---------------------------------------------------------------------------
# Enemy obstacle
# ---------------------------------------------------------------------------


class TestEnemyObstacle:
    def test_enemy_obstacle_shape(self, manager: ObstacleManager):
        cx, cy = 1.5, 1.0
        obs = manager._build_enemy_obstacle(cx, cy)
        xs = sorted(obs.x_list[:-1])
        ys = sorted(obs.y_list[:-1])
        assert abs(xs[0] - (cx - ENEMY_R)) < 1e-9
        assert abs(xs[-1] - (cx + ENEMY_R)) < 1e-9
        assert abs(ys[0] - (cy - ENEMY_R)) < 1e-9
        assert abs(ys[-1] - (cy + ENEMY_R)) < 1e-9

    def test_enemy_obstacle_is_square(self, manager: ObstacleManager):
        obs = manager._build_enemy_obstacle(1.5, 1.0)
        xs = sorted(obs.x_list[:-1])
        ys = sorted(obs.y_list[:-1])
        assert abs((xs[-1] - xs[0]) - (ys[-1] - ys[0])) < 1e-9


# ---------------------------------------------------------------------------
# Enemy pose state
# ---------------------------------------------------------------------------


class TestEnemyPose:
    def test_no_enemy_initially(self, manager: ObstacleManager):
        assert manager.get_dynamic_obstacles() == []

    def test_set_enemy_pose_adds_obstacle(self, manager: ObstacleManager):
        manager.set_enemy_pose(1.5, 1.0)
        assert len(manager.get_dynamic_obstacles()) == 1

    def test_clear_enemy_pose_removes_obstacle(self, manager: ObstacleManager):
        manager.set_enemy_pose(1.5, 1.0)
        manager.clear_enemy_pose()
        assert manager.get_dynamic_obstacles() == []

    def test_enemy_in_get_all_obstacles(self, manager: ObstacleManager):
        base_count = len(manager.get_all_obstacles())
        manager.set_enemy_pose(1.5, 1.0)
        assert len(manager.get_all_obstacles()) == base_count + 1

    def test_enemy_outside_table_filtered_from_dynamic(self, manager: ObstacleManager):
        # Enemy very far outside the table should be filtered out
        manager.set_enemy_pose(100.0, 100.0)
        assert manager.get_dynamic_obstacles() == []


# ---------------------------------------------------------------------------
# get_static_obstacles
# ---------------------------------------------------------------------------


class TestGetStaticObstacles:
    def test_includes_borders(self, manager: ObstacleManager):
        # 4 borders + 1 zone (forbidden_zone_1) + 2 elements
        obstacles = manager.get_static_obstacles()
        assert len(obstacles) >= 4

    def test_total_count_matches_expectations(self, manager: ObstacleManager):
        # 4 borders + 1 occupied zone + 2 elements = 7
        obstacles = manager.get_static_obstacles()
        assert len(obstacles) == 7

    def test_filtered_to_table_area(self, manager: ObstacleManager):
        """No obstacle should be completely outside the table + margin."""
        margin = ROBOT_R
        for obs in manager.get_static_obstacles():
            xs = obs.x_list[:-1]
            ys = obs.y_list[:-1]
            assert max(xs) >= -margin
            assert min(xs) <= TABLE_W + margin
            assert max(ys) >= -margin
            assert min(ys) <= TABLE_H + margin


# ---------------------------------------------------------------------------
# is_point_in_forbidden_area
# ---------------------------------------------------------------------------


class TestIsPointInForbiddenArea:
    """
    expand_distance = ROBOT_R - MARGIN = 0.10 m.
    The left border sits at x ∈ [-0.01, 0.0]; expanded rightward into the table
    by ≈ 0.10 m → the C-space boundary is at x ≈ 0.10.
    """

    def test_midtable_point_is_safe(self, manager: ObstacleManager):
        # (0.5, 1.5) is far from all borders, zones and elements
        assert not manager.is_point_in_forbidden_area(0.5, 1.5)

    def test_point_near_left_wall_is_forbidden(self, manager: ObstacleManager):
        # x=0.05 is inside the 0.10 m expansion of the left border
        assert manager.is_point_in_forbidden_area(0.05, 1.0)

    def test_point_near_bottom_wall_is_forbidden(self, manager: ObstacleManager):
        assert manager.is_point_in_forbidden_area(1.5, 0.05)

    def test_point_past_expansion_is_safe(self, manager: ObstacleManager):
        # x=0.13 > 0.10 expansion → should be outside the expanded left border
        assert not manager.is_point_in_forbidden_area(0.13, 1.0)

    def test_point_in_occupied_zone_is_forbidden(self, manager: ObstacleManager):
        # forbidden_zone_1 is 0.5×0.5 at (1.0, 0.5); centre (1.25, 0.75) ± expansion
        assert manager.is_point_in_forbidden_area(1.25, 0.75)

    def test_point_outside_occupied_zone_is_safe(self, manager: ObstacleManager):
        # Well away from all zones/elements and borders
        assert not manager.is_point_in_forbidden_area(0.5, 1.5)

    def test_after_freeing_zone_point_becomes_safe(self, manager: ObstacleManager):
        # With forbidden_zone_1 freed, its centre should become safe
        manager.update_states({"forbidden_zone_1": False})
        assert not manager.is_point_in_forbidden_area(1.25, 0.75)


# ---------------------------------------------------------------------------
# find_nearest_exit_point
# ---------------------------------------------------------------------------


class TestFindNearestExitPoint:
    def test_returns_none_when_safe(self, manager: ObstacleManager):
        # (0.5, 1.5) is far from all borders, zones and elements
        result = manager.find_nearest_exit_point(0.5, 1.5)
        assert result is None

    def test_returns_point_when_inside_forbidden(self, manager: ObstacleManager):
        result = manager.find_nearest_exit_point(0.05, 1.0)
        assert result is not None

    def test_exit_point_is_outside_forbidden_area(self, manager: ObstacleManager):
        """The returned exit point must itself be outside the forbidden area."""
        x, y = 0.05, 1.0
        result = manager.find_nearest_exit_point(x, y)
        assert result is not None
        exit_x, exit_y = result
        assert not manager.is_point_in_forbidden_area(exit_x, exit_y)

    def test_exit_point_moves_away_from_left_wall(self, manager: ObstacleManager):
        """When stuck near the left wall, exit_x should be larger than current x."""
        result = manager.find_nearest_exit_point(0.05, 1.0)
        assert result is not None
        exit_x, _ = result
        assert exit_x > 0.05

    def test_exit_point_in_zone_is_outside(self, manager: ObstacleManager):
        """A point at the centre of forbidden_zone_1 should get an exit point."""
        # forbidden_zone_1 at (1.0, 0.5), w=0.5, h=0.5 → centre (1.25, 0.75)
        result = manager.find_nearest_exit_point(1.25, 0.75)
        assert result is not None
        exit_x, exit_y = result
        assert not manager.is_point_in_forbidden_area(exit_x, exit_y)

    def test_exit_point_is_tuple_of_two_floats(self, manager: ObstacleManager):
        result = manager.find_nearest_exit_point(0.05, 1.0)
        assert result is not None
        assert len(result) == 2
        assert all(isinstance(v, float) for v in result)


# ---------------------------------------------------------------------------
# Dependency injection — custom forbidden_area_checker
# ---------------------------------------------------------------------------


class TestDependencyInjection:
    def test_custom_checker_is_used(self):
        """A checker with zero expansion should never flag anything as forbidden."""
        no_expand_checker = VisibilityRoadMap(expand_distance=0.0)
        mgr = ObstacleManager(
            config_path=str(FIXTURE),
            table_width=TABLE_W,
            table_height=TABLE_H,
            robot_radius=ROBOT_R,
            enemy_robot_radius=ENEMY_R,
            forbidden_area_margin=MARGIN,
            forbidden_area_checker=no_expand_checker,
        )
        # With zero expansion the borders have zero C-space thickness —
        # points inside the raw wall polygon (x ∈ [-0.01, 0]) would be flagged,
        # but x=0.01 (just inside the table) should be safe
        assert not mgr.is_point_in_forbidden_area(0.01, 1.0)
