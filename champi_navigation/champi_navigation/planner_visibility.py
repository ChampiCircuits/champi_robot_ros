from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional


from champi_interfaces.action import Navigate
from champi_libraries_py.marker_helper import items, presets
from champi_libraries_py.marker_helper.canva import Canva
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.timeout import Timeout
from champi_navigation.obstacle_manager import ObstacleManager
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap
from champi_navigation.planner_status import PlannerStatus
import champi_navigation.goal_checker as goal_checker





@dataclass
class StepOutput:
    """All information the node needs to react to one Planner tick."""

    status: PlannerStatus

    # Movement command — None means "don't send a new ctrl goal this tick"
    ctrl_goal: Optional[Pose2D] = None
    is_waypoint: bool = False   # True when ctrl_goal is an intermediate waypoint

    # Node should call publish_stop() when True
    send_stop: bool = False

    # Feedback / viz data (only meaningful when status is RUNNING or NO_PATH)
    path_result: Optional[Navigate.Feedback] = None
    remaining_path: list[Pose2D] = field(default_factory=list)
    max_linear_speed: float = 0.0

    # Visualisation data (only set when status is RUNNING)
    waypoints: list[Pose2D] = field(default_factory=list)
    waypoint_idx: int = 0



class _ForbiddenAreaPhase(Enum):
    NONE = auto()
    WAITING = auto()   # robot stopped, waiting forbidden_area_wait_time
    EXITING = auto()   # ctrl_goal sent to exit_point, robot moving out


@dataclass
class PlannerDebugGeometry:
    obstacles: list[list[tuple[float, float]]]
    expanded_obstacles: list[list[tuple[float, float]]]


class PlanningMetrics:
    """Tracks planning call statistics, independent of any framework."""

    def __init__(self) -> None:
        self.last_ms: Optional[float] = None
        self.worst_ms: float = 0.0
        self.total_ms: float = 0.0
        self.n_calls: int = 0
        self.n_failed: int = 0

    def record(self, planning_ms: float, success: bool) -> None:
        self.last_ms = planning_ms
        self.n_calls += 1
        self.total_ms += planning_ms
        self.worst_ms = max(self.worst_ms, planning_ms)
        if not success:
            self.n_failed += 1

    @property
    def avg_ms(self) -> float:
        return self.total_ms / self.n_calls if self.n_calls else 0.0

    @property
    def fail_rate(self) -> float:
        return self.n_failed / self.n_calls if self.n_calls else 0.0


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


class PlannerVisibility:
    """Step-based path-planning state machine, free of ROS2 runtime dependencies.

    Usage::

        planner = Planner(obstacle_manager, visibility_planner, ...)
        planner.start(navigate_goal)

        # inside the node loop:
        output = planner.step(robot_pose)
        # react to output.status, output.ctrl_goal, output.send_stop …

    Dependency injection:
    - ``obstacle_manager``: :class:`ObstacleManager` instance (owns all obstacle geometry).
    - ``visibility_planner``: :class:`VisibilityRoadMap` used for path planning.
    """

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        visibility_planner: VisibilityRoadMap,
        waypoint_tolerance: float,
        waypoint_speed_linear: float,
        forbidden_area_wait_time: float,
    ) -> None:
        self._obstacle_manager = obstacle_manager
        self._visibility_planner = visibility_planner
        self._waypoint_tolerance = waypoint_tolerance
        self._waypoint_speed_linear = waypoint_speed_linear
        self._forbidden_area_wait_time = forbidden_area_wait_time

        self.planning_metrics = PlanningMetrics()

        # Navigation state
        self._goal: Optional[Navigate.Goal] = None
        self._goal_pose: Optional[Pose2D] = None
        self._waypoints: Optional[list[Pose2D]] = None
        self._waypoint_idx: int = 1

        # Control flags
        self._cancelled: bool = False

        # Timers
        self._nav_timeout = Timeout()
        self._forbidden_wait_timeout = Timeout()

        # Forbidden-area state machine
        self._forbidden_phase: _ForbiddenAreaPhase = _ForbiddenAreaPhase.NONE
        self._exit_point: Optional[Pose2D] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def start(self, goal: Navigate.Goal) -> None:
        """Begin navigation towards *goal*, resetting all state."""
        self._goal = goal
        self._goal_pose = Pose2D(pose=goal.pose)
        self._waypoints = None
        self._waypoint_idx = 1
        self._cancelled = False
        self._forbidden_phase = _ForbiddenAreaPhase.NONE
        self._exit_point = None
        self._nav_timeout.start(goal.timeout)

    def cancel(self) -> None:
        """Signal that the current goal should be cancelled."""
        self._cancelled = True

    def update_obstacle_states(self, states: dict[str, bool]) -> bool:
        """Forward brain obstacle-state snapshot to ObstacleManager.

        Returns ``True`` if any state changed (caller may wish to log this).
        Invalidates the VisibilityRoadMap cache automatically when needed.
        """
        changed = self._obstacle_manager.update_states(states)
        if changed:
            self._visibility_planner.invalidate_cache()
        return changed

    def set_enemy_pose(self, x: float, y: float) -> None:
        """Update enemy robot position in the ObstacleManager."""
        self._obstacle_manager.set_enemy_pose(x, y)

    # ------------------------------------------------------------------
    # Path computation (private)
    # ------------------------------------------------------------------

    def _compute_path(self, start: Pose2D, goal: Pose2D) -> Optional[list[Pose2D]]:
        """Run the visibility-road-map planner and record diagnostics.

        Returns a list of :class:`Pose2D` waypoints (start inclusive) or
        ``None`` when no path exists.
        """
        static_obstacles = self._obstacle_manager.get_static_obstacles()
        dynamic_obstacles = self._obstacle_manager.get_dynamic_obstacles()

        rx, ry, planning_ms = self._visibility_planner.planning(
            start.x, start.y, goal.x, goal.y, static_obstacles, dynamic_obstacles
        )

        if not rx or not ry:
            self.planning_metrics.record(planning_ms, success=False)
            return None

        self.planning_metrics.record(planning_ms, success=True)

        # rx/ry are goal→start; reverse to start→goal
        rx.reverse()
        ry.reverse()
        return [Pose2D(x=float(x), y=float(y), theta=goal.theta) for x, y in zip(rx, ry)]

    # ------------------------------------------------------------------
    # Step — core state machine
    # ------------------------------------------------------------------

    def step(self, robot_pose: Optional[Pose2D]) -> StepOutput:
        """Advance the planner by one tick.

        Args:
            robot_pose: Current robot pose, or ``None`` if not yet received.

        Returns:
            :class:`StepOutput` describing what the node should do this tick.
        """
        # 1. No active goal
        if self._goal is None:
            return StepOutput(status=PlannerStatus.IDLE)

        # 2. Waiting for first odometry
        if robot_pose is None:
            return StepOutput(
                status=PlannerStatus.INITIALIZING,
                path_result=Navigate.Feedback.INTITIALIZING,
            )

        # 3. Cancelled
        if self._cancelled:
            return StepOutput(status=PlannerStatus.CANCELLED, send_stop=True)

        # 4. Navigation timeout
        if self._nav_timeout.is_elapsed():
            return StepOutput(status=PlannerStatus.TIMED_OUT, send_stop=True)

        # 5-8. Forbidden-area state machine (only when collision avoidance is active)
        if self._goal.use_collision_avoidance and self._forbidden_phase == _ForbiddenAreaPhase.WAITING:
            if not self._forbidden_wait_timeout.is_elapsed():
                # Still waiting; keep robot stopped
                return StepOutput(status=PlannerStatus.IN_FORBIDDEN_AREA, send_stop=True)
            # Wait elapsed — compute exit point, transition to EXITING
            result = self._obstacle_manager.find_nearest_exit_point(robot_pose.x, robot_pose.y)
            if result is not None:
                exit_x, exit_y = result
                self._exit_point = Pose2D(x=exit_x, y=exit_y, theta=robot_pose.theta)
            else:
                self._exit_point = None
            self._forbidden_phase = _ForbiddenAreaPhase.EXITING
            return StepOutput(
                status=PlannerStatus.IN_FORBIDDEN_AREA,
                ctrl_goal=self._exit_point,
                is_waypoint=True,
            )

        if self._goal.use_collision_avoidance and self._forbidden_phase == _ForbiddenAreaPhase.EXITING:
            if self._obstacle_manager.is_point_in_forbidden_area(robot_pose.x, robot_pose.y):
                # Robot still inside — keep sending exit ctrl goal
                return StepOutput(
                    status=PlannerStatus.IN_FORBIDDEN_AREA,
                    ctrl_goal=self._exit_point,
                    is_waypoint=True,
                )
            # Robot has exited — resume normal planning
            self._forbidden_phase = _ForbiddenAreaPhase.NONE
            self._exit_point = None
            # Reset waypoints so path is recomputed fresh this tick

        # 9. Check if robot entered a forbidden area this tick
        if self._obstacle_manager.is_point_in_forbidden_area(robot_pose.x, robot_pose.y):
            self._forbidden_phase = _ForbiddenAreaPhase.WAITING
            self._forbidden_wait_timeout.start(self._forbidden_area_wait_time)
            self._waypoints = None  # will replan after exiting
            return StepOutput(status=PlannerStatus.IN_FORBIDDEN_AREA, send_stop=True)

        # 10. Pre-check: robot already at goal (avoids NO_PATH when start == goal)
        if goal_checker.is_goal_reached(
            self._goal_pose,
            robot_pose,
            self._goal.end_speed == 0,
            self._goal.do_look_at_point,
            Pose2D(point=self._goal.look_at_point),
            self._goal.robot_angle_when_looking_at_point,
            self._goal.linear_tolerance,
            self._goal.angular_tolerance,
        ):
            self._goal = None
            return StepOutput(status=PlannerStatus.GOAL_REACHED, send_stop=True)

        # Recompute path (replans every tick to react to dynamic obstacles)
        new_waypoints = self._compute_path(robot_pose, self._goal_pose)
        if new_waypoints is not None:
            self._waypoints = new_waypoints
            self._waypoint_idx = 1  # index 0 is always the robot's current position

        # 11. No valid path
        if self._waypoints is None or len(self._waypoints) < 2:
            return StepOutput(
                status=PlannerStatus.NO_PATH,
                path_result=Navigate.Feedback.NO_PATH_FOUND,
                max_linear_speed=self._goal.max_linear_speed,
            )

        # 12. Waypoint tracking
        target_wp = self._waypoints[self._waypoint_idx]
        is_last = self._waypoint_idx >= len(self._waypoints) - 1

        look_at = (
            Pose2D(point=self._goal.look_at_point) if is_last else target_wp
        )
        angle_when_look = self._goal.robot_angle_when_looking_at_point if is_last else 0.0
        lin_tol = self._goal.linear_tolerance if is_last else self._waypoint_tolerance
        ang_tol = self._goal.angular_tolerance if is_last else 3.14

        if goal_checker.is_goal_reached(
            target_wp,
            robot_pose,
            self._goal.end_speed == 0 and is_last,
            self._goal.do_look_at_point and is_last,
            look_at,
            angle_when_look,
            lin_tol,
            ang_tol,
        ):
            if is_last:
                self._goal = None  # disarm for next call
                return StepOutput(status=PlannerStatus.GOAL_REACHED, send_stop=True)
            # Advance to next waypoint
            self._waypoint_idx += 1
            target_wp = self._waypoints[self._waypoint_idx]
            is_last = self._waypoint_idx >= len(self._waypoints) - 1

        # 13. Normal running tick
        path_result = (
            Navigate.Feedback.SUCCESS_STRAIGHT
            if len(self._waypoints) == 2
            else Navigate.Feedback.SUCCESS_AVOIDANCE
        )
        remaining_path = [robot_pose] + self._waypoints[self._waypoint_idx:]

        return StepOutput(
            status=PlannerStatus.RUNNING,
            ctrl_goal=target_wp,
            is_waypoint=not is_last,
            path_result=path_result,
            remaining_path=remaining_path,
            max_linear_speed=self._goal.max_linear_speed,
            waypoints=self._waypoints,
            waypoint_idx=self._waypoint_idx,
        )


    @property
    def has_goal(self) -> bool:
        return self._goal is not None
    


    def draw_viz(self, waypoints: Optional[List[Pose2D]], current_waypoint_idx: int) -> None:
        Canva().clear()
        
        raw_obstacles = self._obstacle_manager.get_all_obstacles()
        expanded_obstacles = self._visibility_planner.build_expanded_obstacles(raw_obstacles)

        obstacles=[list(zip(obs.x_list, obs.y_list)) for obs in raw_obstacles]
        expanded_obstacles=[list(zip(obs.x_list, obs.y_list)) for obs in expanded_obstacles]
        
        for points in obstacles:
            Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.RED), frame_id='odom')

        for points in expanded_obstacles:
            Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.ORANGE), frame_id='odom')

        if waypoints and current_waypoint_idx < len(waypoints):
            path_points = [(wp.x, wp.y) for wp in waypoints[current_waypoint_idx:]]
            if len(path_points) >= 2:
                Canva().add(items.Polyline(path_points, size=presets.LINE_MEDIUM, color=presets.GREEN), frame_id='odom')
            Canva().add(items.Spheres(path_points, color=presets.CYAN), frame_id='odom')

        Canva().draw()