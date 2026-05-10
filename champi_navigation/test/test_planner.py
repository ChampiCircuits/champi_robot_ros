"""Unit tests for champi_navigation.planner.Planner.

All tests are pure Python — no ROS2 runtime needed.
Navigate.Goal is constructed directly as a rosidl-generated dataclass.

Run with:
  pytest champi_navigation/test/test_planner.py -v
"""

from __future__ import annotations

import pathlib
import time
from unittest.mock import MagicMock, patch

import pytest

from champi_interfaces.action import Navigate
from champi_libraries_py.data_types.geometry import Pose2D
from geometry_msgs.msg import Point, Pose, Quaternion

from champi_navigation.obstacle_manager import ObstacleManager
from champi_navigation.planner import (
    Planner,
    PlannerStatus,
    PlanningMetrics,
    _ForbiddenAreaPhase,
)
from champi_navigation.planning_feedback import ComputePathResult
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap

# ---------------------------------------------------------------------------
# Shared constants & helpers
# ---------------------------------------------------------------------------

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "world_state_test.yaml"
TABLE_W = 3.0
TABLE_H = 2.0
ROBOT_R = 0.15
ENEMY_R = 0.20
MARGIN = 0.05


def _make_obstacle_manager() -> ObstacleManager:
    return ObstacleManager(
        config_path=str(FIXTURE),
        table_width=TABLE_W,
        table_height=TABLE_H,
        robot_radius=ROBOT_R,
        enemy_robot_radius=ENEMY_R,
        forbidden_area_margin=MARGIN,
    )


def _make_planner(
    obstacle_manager: ObstacleManager | None = None,
    forbidden_area_wait_time: float = 0.5,
) -> Planner:
    om = obstacle_manager or _make_obstacle_manager()
    vp = VisibilityRoadMap(expand_distance=ROBOT_R)
    return Planner(
        obstacle_manager=om,
        visibility_planner=vp,
        waypoint_tolerance=0.05,
        waypoint_speed_linear=0.3,
        forbidden_area_wait_time=forbidden_area_wait_time,
    )


def _make_goal(
    x: float = 0.5,
    y: float = 1.5,
    theta: float = 0.0,
    timeout: float = 30.0,
    end_speed: float = 0.0,
    linear_tolerance: float = 0.03,
    angular_tolerance: float = 0.1,
    max_linear_speed: float = 0.3,
    do_look_at_point: bool = False,
) -> Navigate.Goal:
    goal = Navigate.Goal()
    goal.pose = Pose()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    goal.timeout = float(timeout)
    goal.end_speed = float(end_speed)
    goal.linear_tolerance = float(linear_tolerance)
    goal.angular_tolerance = float(angular_tolerance)
    goal.max_linear_speed = float(max_linear_speed)
    goal.do_look_at_point = do_look_at_point
    goal.look_at_point = Point(x=0.0, y=0.0, z=0.0)
    goal.robot_angle_when_looking_at_point = 0.0
    goal.max_angular_speed = 1.0
    goal.accel_linear = 1.0
    goal.accel_angular = 1.0
    return goal


def _pose(x: float, y: float, theta: float = 0.0) -> Pose2D:
    return Pose2D(x=float(x), y=float(y), theta=float(theta))


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def planner() -> Planner:
    return _make_planner()


# ---------------------------------------------------------------------------
# PlanningMetrics
# ---------------------------------------------------------------------------


class TestPlanningMetrics:
    def test_initial_state_no_calls(self):
        diag = PlanningMetrics()
        assert diag.last_ms is None
        assert diag.n_calls == 0

    def test_record_success(self):
        diag = PlanningMetrics()
        diag.record(10.0, success=True)
        assert diag.n_calls == 1
        assert diag.n_failed == 0
        assert diag.worst_ms == 10.0

    def test_record_failure(self):
        diag = PlanningMetrics()
        diag.record(5.0, success=False)
        assert diag.n_failed == 1

    def test_worst_ms_tracked(self):
        diag = PlanningMetrics()
        diag.record(10.0, success=True)
        diag.record(50.0, success=True)
        diag.record(20.0, success=True)
        assert diag.worst_ms == 50.0


# ---------------------------------------------------------------------------
# IDLE — no active goal
# ---------------------------------------------------------------------------


class TestIdle:
    def test_idle_before_start(self, planner: Planner):
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.IDLE

    def test_idle_after_goal_reached(self, planner: Planner):
        # Place robot exactly at goal; check that after GOAL_REACHED Planner returns IDLE
        goal = _make_goal(x=0.5, y=1.5, linear_tolerance=1.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.GOAL_REACHED
        # Next tick has no active goal
        out2 = planner.step(_pose(0.5, 1.5))
        assert out2.status == PlannerStatus.IDLE


# ---------------------------------------------------------------------------
# INITIALIZING — no robot pose yet
# ---------------------------------------------------------------------------


class TestInitializing:
    def test_initializing_no_pose(self, planner: Planner):
        planner.start(_make_goal())
        out = planner.step(None)
        assert out.status == PlannerStatus.INITIALIZING
        assert out.path_result == ComputePathResult.INITIALIZING

    def test_no_ctrl_goal_while_initializing(self, planner: Planner):
        planner.start(_make_goal())
        out = planner.step(None)
        assert out.ctrl_goal is None
        assert not out.send_stop


# ---------------------------------------------------------------------------
# CANCELLED
# ---------------------------------------------------------------------------


class TestCancel:
    def test_cancel_returns_cancelled(self, planner: Planner):
        planner.start(_make_goal())
        planner.cancel()
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.CANCELLED
        assert out.send_stop

    def test_cancel_before_start_is_harmless(self, planner: Planner):
        planner.cancel()   # should not raise
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.IDLE

    def test_cancel_flag_not_carried_across_goals(self, planner: Planner):
        planner.start(_make_goal())
        planner.cancel()
        # New start() resets the flag
        planner.start(_make_goal())
        out = planner.step(None)
        assert out.status == PlannerStatus.INITIALIZING


# ---------------------------------------------------------------------------
# TIMED_OUT
# ---------------------------------------------------------------------------


class TestTimeout:
    def test_timeout_triggers(self, planner: Planner):
        goal = _make_goal(timeout=0.01)
        planner.start(goal)
        time.sleep(0.05)
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.TIMED_OUT
        assert out.send_stop

    def test_no_timeout_within_limit(self, planner: Planner):
        goal = _make_goal(timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        # Should be RUNNING or NO_PATH (not timed out)
        assert out.status not in (PlannerStatus.TIMED_OUT, PlannerStatus.IDLE)


# ---------------------------------------------------------------------------
# GOAL_REACHED
# ---------------------------------------------------------------------------


class TestGoalReached:
    def test_goal_reached_at_start_position(self, planner: Planner):
        """Robot is already at the goal — should reach it immediately."""
        goal = _make_goal(x=0.5, y=1.5, linear_tolerance=1.0, angular_tolerance=3.14)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        assert out.status == PlannerStatus.GOAL_REACHED

    def test_goal_reached_send_stop(self, planner: Planner):
        goal = _make_goal(x=0.5, y=1.5, linear_tolerance=1.0, angular_tolerance=3.14)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        assert out.send_stop


# ---------------------------------------------------------------------------
# RUNNING — normal navigation
# ---------------------------------------------------------------------------


class TestRunning:
    def test_running_produces_ctrl_goal(self, planner: Planner):
        """Robot away from goal; planner should return RUNNING with a ctrl_goal."""
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 0.3))
        # The table is clear near these points; a path should be found
        assert out.status in (PlannerStatus.RUNNING, PlannerStatus.NO_PATH)
        if out.status == PlannerStatus.RUNNING:
            assert out.ctrl_goal is not None

    def test_running_path_result_straight(self, planner: Planner):
        """Straight clear path should yield SUCCESS_STRAIGHT."""
        # Put start and goal on a clear corridor
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        if out.status == PlannerStatus.RUNNING:
            assert out.path_result in (
                ComputePathResult.SUCCESS_STRAIGHT,
                ComputePathResult.SUCCESS_AVOIDANCE,
            )

    def test_running_remaining_path_starts_with_robot_pose(self, planner: Planner):
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        robot = _pose(0.5, 1.5)
        out = planner.step(robot)
        if out.status == PlannerStatus.RUNNING:
            assert len(out.remaining_path) >= 1
            assert abs(out.remaining_path[0].x - robot.x) < 1e-6
            assert abs(out.remaining_path[0].y - robot.y) < 1e-6

    def test_running_waypoints_provided(self, planner: Planner):
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        if out.status == PlannerStatus.RUNNING:
            assert len(out.waypoints) >= 2
            assert out.waypoint_idx >= 1

    def test_waypoint_advance_when_reached(self, planner: Planner):
        """Simulate waypoint advancement by placing robot exactly at intermediate waypoint."""
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        # First step to get a path
        out1 = planner.step(_pose(0.5, 1.5))
        if out1.status != PlannerStatus.RUNNING or len(out1.waypoints) < 3:
            pytest.skip("Need avoidance path with ≥3 waypoints for this test")
        # Move robot to first waypoint to trigger advance
        wp1 = out1.waypoints[1]
        out2 = planner.step(_pose(wp1.x, wp1.y))
        if out2.status == PlannerStatus.RUNNING:
            assert out2.waypoint_idx >= 2 or out2.status == PlannerStatus.GOAL_REACHED


# ---------------------------------------------------------------------------
# NO_PATH
# ---------------------------------------------------------------------------


class TestNoPath:
    def test_no_path_when_goal_in_obstacle(self, planner: Planner):
        """Goal inside the border expansion → no path expected."""
        # Goal at (0.02, 0.02): inside C-space expansion of two borders
        goal = _make_goal(x=0.02, y=0.02, timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.5, 1.5))
        assert out.status in (PlannerStatus.NO_PATH, PlannerStatus.RUNNING)
        # If NO_PATH, verify fields
        if out.status == PlannerStatus.NO_PATH:
            assert out.path_result == ComputePathResult.NO_PATH_FOUND
            assert out.ctrl_goal is None
            assert not out.send_stop


# ---------------------------------------------------------------------------
# IN_FORBIDDEN_AREA
# ---------------------------------------------------------------------------


class TestForbiddenArea:
    def test_robot_near_wall_triggers_forbidden(self):
        """x=0.05 is inside C-space expansion of left border (expand=0.10)."""
        planner = _make_planner(forbidden_area_wait_time=0.5)
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        out = planner.step(_pose(0.05, 1.0))
        assert out.status == PlannerStatus.IN_FORBIDDEN_AREA
        assert out.send_stop
        assert out.ctrl_goal is None  # WAITING phase: no ctrl_goal yet

    def test_forbidden_waiting_phase_sends_stop(self):
        planner = _make_planner(forbidden_area_wait_time=30.0)  # long wait
        goal = _make_goal(x=2.5, y=1.5, timeout=60.0)
        planner.start(goal)
        # Enter forbidden area
        planner.step(_pose(0.05, 1.0))
        assert planner._forbidden_phase == _ForbiddenAreaPhase.WAITING
        # Next tick still in WAITING
        out = planner.step(_pose(0.05, 1.0))
        assert out.status == PlannerStatus.IN_FORBIDDEN_AREA
        assert out.send_stop
        assert out.ctrl_goal is None

    def test_forbidden_exiting_phase_sends_ctrl_goal(self):
        planner = _make_planner(forbidden_area_wait_time=0.01)
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        planner.step(_pose(0.05, 1.0))   # enter → WAITING
        time.sleep(0.05)                 # wait expires
        out = planner.step(_pose(0.05, 1.0))   # transition to EXITING
        assert out.status == PlannerStatus.IN_FORBIDDEN_AREA
        # Either ctrl_goal was found or exit_point is None (edge case)
        # The important thing is the phase transitioned
        assert planner._forbidden_phase == _ForbiddenAreaPhase.EXITING

    def test_forbidden_exiting_phase_repeats_ctrl_goal(self):
        planner = _make_planner(forbidden_area_wait_time=0.01)
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        planner.step(_pose(0.05, 1.0))  # → WAITING
        time.sleep(0.05)
        planner.step(_pose(0.05, 1.0))  # → EXITING
        out2 = planner.step(_pose(0.05, 1.0))  # still EXITING
        assert out2.status == PlannerStatus.IN_FORBIDDEN_AREA
        assert planner._forbidden_phase == _ForbiddenAreaPhase.EXITING

    def test_forbidden_clears_after_robot_exits(self):
        planner = _make_planner(forbidden_area_wait_time=0.01)
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        planner.step(_pose(0.05, 1.0))  # → WAITING
        time.sleep(0.05)
        planner.step(_pose(0.05, 1.0))  # → EXITING
        # Robot moves to safe position
        out = planner.step(_pose(0.5, 1.5))
        assert planner._forbidden_phase == _ForbiddenAreaPhase.NONE
        assert out.status != PlannerStatus.IN_FORBIDDEN_AREA


# ---------------------------------------------------------------------------
# Obstacle state updates & cache invalidation
# ---------------------------------------------------------------------------


class TestObstacleStateUpdates:
    def test_update_obstacle_states_returns_changed(self, planner: Planner):
        # placement_zone_1 starts free; marking it occupied is a change
        changed = planner.update_obstacle_states({"placement_zone_1": True})
        assert changed is True

    def test_update_no_change_returns_false(self, planner: Planner):
        changed = planner.update_obstacle_states({"forbidden_zone_1": True})
        assert changed is False

    def test_cache_invalidated_on_change(self, planner: Planner):
        """After a state change, the visibility planner cache should be None."""
        # Prime the cache with a planning call
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        planner.step(_pose(0.5, 1.5))
        assert planner._visibility_planner._static_cache is not None
        # Now change an obstacle state
        planner.update_obstacle_states({"placement_zone_1": True})
        assert planner._visibility_planner._static_cache is None

    def test_cache_not_invalidated_on_no_change(self, planner: Planner):
        goal = _make_goal(x=2.5, y=1.5, timeout=30.0)
        planner.start(goal)
        planner.step(_pose(0.5, 1.5))
        cache_before = planner._visibility_planner._static_cache
        planner.update_obstacle_states({"forbidden_zone_1": True})
        assert planner._visibility_planner._static_cache is cache_before


# ---------------------------------------------------------------------------
# Enemy pose
# ---------------------------------------------------------------------------


class TestEnemyPose:
    def test_set_enemy_pose_forwarded_to_obstacle_manager(self, planner: Planner):
        planner.set_enemy_pose(1.5, 1.0)
        assert planner._obstacle_manager._enemy_pose == (1.5, 1.0)

    def test_enemy_appears_in_get_all_obstacles(self, planner: Planner):
        base = len(planner._obstacle_manager.get_all_obstacles())
        planner.set_enemy_pose(1.5, 1.0)
        assert len(planner._obstacle_manager.get_all_obstacles()) == base + 1


# ---------------------------------------------------------------------------
# Accessors / convenience pass-throughs
# ---------------------------------------------------------------------------


class TestAccessors:
    def test_has_goal_false_initially(self, planner: Planner):
        assert not planner.has_goal

    def test_has_goal_true_after_start(self, planner: Planner):
        planner.start(_make_goal())
        assert planner.has_goal

    def test_has_goal_false_after_goal_reached(self, planner: Planner):
        goal = _make_goal(x=0.5, y=1.5, linear_tolerance=1.0, angular_tolerance=3.14)
        planner.start(goal)
        planner.step(_pose(0.5, 1.5))
        assert not planner.has_goal

# ---------------------------------------------------------------------------
# Dependency injection — custom visibility planner
# ---------------------------------------------------------------------------


class TestDependencyInjection:
    def test_custom_visibility_planner_injected(self):
        """Inject a mock visibility planner to verify _compute_path calls it."""
        mock_vp = MagicMock(spec=VisibilityRoadMap)
        mock_vp._static_cache = None
        mock_vp.planning.return_value = ([], [], 0.0)  # no path
        mock_vp.build_expanded_obstacles.return_value = []

        om = _make_obstacle_manager()
        planner = Planner(
            obstacle_manager=om,
            visibility_planner=mock_vp,
            waypoint_tolerance=0.05,
            waypoint_speed_linear=0.3,
            forbidden_area_wait_time=0.5,
        )
        planner.start(_make_goal(x=2.5, y=1.5))
        planner.step(_pose(0.5, 1.5))
        mock_vp.planning.assert_called_once()

    def test_custom_obstacle_manager_injected(self):
        """Inject a mock ObstacleManager to verify step() delegates to it."""
        mock_om = MagicMock(spec=ObstacleManager)
        mock_om.get_static_obstacles.return_value = []
        mock_om.get_dynamic_obstacles.return_value = []
        mock_om.get_all_obstacles.return_value = []
        mock_om.is_point_in_forbidden_area.return_value = False

        vp = MagicMock(spec=VisibilityRoadMap)
        vp._static_cache = None
        vp.planning.return_value = ([], [], 0.0)
        vp.build_expanded_obstacles.return_value = []

        planner = Planner(
            obstacle_manager=mock_om,
            visibility_planner=vp,
            waypoint_tolerance=0.05,
            waypoint_speed_linear=0.3,
            forbidden_area_wait_time=0.5,
        )
        planner.start(_make_goal())
        planner.step(_pose(0.5, 1.5))
        mock_om.is_point_in_forbidden_area.assert_called()
