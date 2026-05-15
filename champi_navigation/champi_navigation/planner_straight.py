from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

from champi_interfaces.action import Navigate
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.timeout import Timeout
import champi_navigation.goal_checker as goal_checker
from champi_navigation.planner_status import PlannerStatus

@dataclass
class StepOutput:
    status: PlannerStatus
    ctrl_goal: Optional[Pose2D] = None
    is_waypoint: bool = False
    send_stop: bool = False
    path_result: Optional[Navigate.Feedback] = None
    remaining_path: list[Pose2D] = field(default_factory=list)
    max_linear_speed: float = 0.0
    waypoints: list[Pose2D] = field(default_factory=list)
    waypoint_idx: int = 0


class PlanningMetrics:
    def __init__(self) -> None:
        self.last_ms: Optional[float] = None
        self.worst_ms: float = 0.0
        self.total_ms: float = 0.0
        self.n_calls: int = 0
        self.n_failed: int = 0

    def record(self, planning_ms: float, success: bool) -> None:
        pass

    @property
    def avg_ms(self) -> float:
        return 0.0

    @property
    def fail_rate(self) -> float:
        return 0.0


class _CollisionPhase(Enum):
    NORMAL = auto()
    WAITING_BEFORE_BACK = auto()
    STEPPING_BACK = auto()


class PlannerStraight:
    def __init__(
        self,
        waypoint_tolerance: float,
        enemy_front_max_ahead_distance_m: float,
        enemy_front_max_lateral_offset_m: float,
    ) -> None:
        self._waypoint_tolerance = waypoint_tolerance
        self._enemy_front_max_ahead_distance_m = enemy_front_max_ahead_distance_m
        self._enemy_front_max_lateral_offset_m = enemy_front_max_lateral_offset_m
        
        self.planning_metrics = PlanningMetrics()

        self._goal: Optional[Navigate.Goal] = None
        self._goal_pose: Optional[Pose2D] = None
        self._cancelled: bool = False
        self._nav_timeout = Timeout()

        self._enemy_pose: Optional[tuple[float, float]] = None
        
        self._collision_phase: _CollisionPhase = _CollisionPhase.NORMAL
        self._step_back_goal: Optional[Pose2D] = None
        self._step_back_wait_timeout = Timeout()
        self._step_back_wait_s: float = 1.0
        self._step_back_distance_m: float = 0.10

    def start(self, goal: Navigate.Goal) -> None:
        self._goal = goal
        self._goal_pose = Pose2D(pose=goal.pose)
        self._cancelled = False
        self._collision_phase = _CollisionPhase.NORMAL
        self._step_back_goal = None
        self._step_back_wait_timeout.reset()
        self._nav_timeout.start(goal.timeout)

    def cancel(self) -> None:
        self._cancelled = True

    def update_obstacle_states(self, states: dict[str, bool]) -> bool:
        return False

    def set_enemy_pose(self, x: float, y: float) -> None:
        self._enemy_pose = (x, y)

    def _is_enemy_in_front(self, robot_pose: Pose2D) -> bool:
        if not self._enemy_pose or self._goal_pose is None:
            return False
            
        ex, ey = self._enemy_pose
        rx, ry = robot_pose.x, robot_pose.y

        # "Front" is defined along the robot->goal line (omnidirectional robot).
        vx_goal = self._goal_pose.x - rx
        vy_goal = self._goal_pose.y - ry
        goal_dist = math.hypot(vx_goal, vy_goal)
        if goal_dist <= 1e-6:
            return False

        ux = vx_goal / goal_dist
        uy = vy_goal / goal_dist

        vx_enemy = ex - rx
        vy_enemy = ey - ry

        along_track = vx_enemy * ux + vy_enemy * uy
        lateral_track = abs(vx_enemy * uy - vy_enemy * ux)

        # Enemy must be ahead on the line and close to it.
        return (
            0.0 < along_track <= self._enemy_front_max_ahead_distance_m
            and lateral_track <= self._enemy_front_max_lateral_offset_m
        )

    def step(self, robot_pose: Optional[Pose2D]) -> StepOutput:
        if self._goal is None:
            return StepOutput(status=PlannerStatus.IDLE)

        if robot_pose is None:
            return StepOutput(
                status=PlannerStatus.INITIALIZING,
                path_result=Navigate.Feedback.INTITIALIZING,
            )

        if self._cancelled:
            return StepOutput(status=PlannerStatus.CANCELLED, send_stop=True)

        if self._nav_timeout.is_elapsed():
            return StepOutput(status=PlannerStatus.TIMED_OUT, send_stop=True)

        if self._collision_phase == _CollisionPhase.NORMAL:
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

            if self._is_enemy_in_front(robot_pose):
                self._collision_phase = _CollisionPhase.WAITING_BEFORE_BACK
                self._step_back_wait_timeout.start(self._step_back_wait_s)
                dx_goal = self._goal_pose.x - robot_pose.x
                dy_goal = self._goal_pose.y - robot_pose.y
                dist_goal = math.hypot(dx_goal, dy_goal)
                if dist_goal > 1e-6:
                    ux = dx_goal / dist_goal
                    uy = dy_goal / dist_goal
                else:
                    ux = math.cos(robot_pose.theta)
                    uy = math.sin(robot_pose.theta)

                # Step back opposite to the robot->goal segment direction.
                bx = robot_pose.x - self._step_back_distance_m * ux
                by = robot_pose.y - self._step_back_distance_m * uy
                self._step_back_goal = Pose2D(x=bx, y=by, theta=robot_pose.theta)
                return StepOutput(
                    status=PlannerStatus.RUNNING,
                    send_stop=True,
                    path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                )

            return StepOutput(
                status=PlannerStatus.RUNNING,
                ctrl_goal=self._goal_pose,
                is_waypoint=False,
                path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                remaining_path=[robot_pose, self._goal_pose],
                max_linear_speed=self._goal.max_linear_speed,
                waypoints=[robot_pose, self._goal_pose],
                waypoint_idx=1,
            )

        elif self._collision_phase == _CollisionPhase.WAITING_BEFORE_BACK:
            if not self._is_enemy_in_front(robot_pose):
                self._collision_phase = _CollisionPhase.NORMAL
                return StepOutput(
                    status=PlannerStatus.RUNNING,
                    ctrl_goal=self._goal_pose,
                    is_waypoint=False,
                    path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                    remaining_path=[robot_pose, self._goal_pose],
                    max_linear_speed=self._goal.max_linear_speed,
                    waypoints=[robot_pose, self._goal_pose],
                    waypoint_idx=1,
                )

            if not self._step_back_wait_timeout.is_elapsed():
                return StepOutput(
                    status=PlannerStatus.RUNNING,
                    send_stop=True,
                    path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                )

            self._collision_phase = _CollisionPhase.STEPPING_BACK
            step_back_cmd_goal = Pose2D(x=self._step_back_goal.x, y=self._step_back_goal.y, theta=robot_pose.theta)
            return StepOutput(
                status=PlannerStatus.RUNNING,
                ctrl_goal=step_back_cmd_goal,
                is_waypoint=True,
                path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                remaining_path=[robot_pose, self._step_back_goal],
                max_linear_speed=self._goal.max_linear_speed,
                waypoints=[robot_pose, self._step_back_goal],
                waypoint_idx=1,
            )

        elif self._collision_phase == _CollisionPhase.STEPPING_BACK:
            if goal_checker.is_goal_reached(
                self._step_back_goal,
                robot_pose,
                True, False, Pose2D(), 0.0,
                self._waypoint_tolerance, 3.14
            ):
                self._collision_phase = _CollisionPhase.NORMAL
                return StepOutput(
                    status=PlannerStatus.RUNNING, 
                    ctrl_goal=robot_pose, 
                    send_stop=True,
                    path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
                )
                
            return StepOutput(
                status=PlannerStatus.RUNNING,
                ctrl_goal=Pose2D(x=self._step_back_goal.x, y=self._step_back_goal.y, theta=robot_pose.theta),
                is_waypoint=True,
                path_result=Navigate.Feedback.SUCCESS_STRAIGHT,
            )

    @property
    def has_goal(self) -> bool:
        return self._goal is not None
    
    def draw_viz(self, waypoints: Optional[List[Pose2D]], current_waypoint_idx: int) -> None:
        pass