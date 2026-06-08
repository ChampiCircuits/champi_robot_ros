#!/usr/bin/env python3
"""Auto-placement control flow for match startup."""

from __future__ import annotations

import time
from math import sqrt
from typing import Callable, Optional


class AutoPlacementController:
    """State machine for startup auto-placement.

    Flow:
    1) Wait robot still for a short duration.
    2) Collect coherent ArUco poses.
    3) Request set_pose from localization.
    4) Request move_to toward strategy init pose.
    """

    STATUS_IDLE = "idle"
    STATUS_WAITING_STILL = "waiting_still"
    STATUS_LOCALIZING = "localizing"
    STATUS_SETTING_POSE = "setting_pose"
    STATUS_MOVING_TO_INIT = "moving_to_init"
    STATUS_FAILED = "failed"

    def __init__(
        self,
        logger,
        request_set_pose: Callable[[list[float]], None],
        request_move_to_init: Callable[[tuple[float, float, float]], None],
        on_completed: Callable[[], None],
        total_timeout_s: float = 15.0,
        rest_required_s: float = 0.5,
        required_aruco_poses: int = 3,
        aruco_pos_threshold_m: float = 0.05,
        aruco_angle_threshold_deg: float = 8.0,
        linear_vel_threshold: float = 0.03,
        angular_vel_threshold: float = 0.15,
    ) -> None:
        self.logger = logger
        self._request_set_pose = request_set_pose
        self._request_move_to_init = request_move_to_init
        self._on_completed = on_completed

        self.total_timeout_s = total_timeout_s
        self.rest_required_s = rest_required_s
        self.required_aruco_poses = required_aruco_poses
        self.aruco_pos_threshold_m = aruco_pos_threshold_m
        self.aruco_angle_threshold_deg = aruco_angle_threshold_deg
        self.linear_vel_threshold = linear_vel_threshold
        self.angular_vel_threshold = angular_vel_threshold

        self.enabled = False
        self.status = self.STATUS_IDLE
        self.in_progress = False
        self._started_at: Optional[float] = None
        self._still_since: Optional[float] = None
        self._aruco_candidates: list[tuple[float, float, float]] = []
        self._pending_init_pose: Optional[tuple[float, float, float]] = None

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def reset(self) -> None:
        self.status = self.STATUS_IDLE
        self.in_progress = False
        self._started_at = None
        self._still_since = None
        self._aruco_candidates = []
        self._pending_init_pose = None

    def start(self, init_pose: tuple[float, float, float]) -> None:
        self._pending_init_pose = init_pose
        self._started_at = time.monotonic()
        self._still_since = None
        self._aruco_candidates = []
        self.status = self.STATUS_WAITING_STILL
        self.in_progress = True
        self.logger.warn("Auto-placement started: waiting for robot to be still")

    def update(self, linear_velocity: float, angular_velocity: float) -> None:
        if not self.in_progress or self.status == self.STATUS_FAILED:
            return

        if self.status == self.STATUS_WAITING_STILL:
            if self._is_robot_at_rest(linear_velocity, angular_velocity):
                if self._still_since is None:
                    self._still_since = time.monotonic()
                elif (time.monotonic() - self._still_since) >= self.rest_required_s:
                    self.status = self.STATUS_LOCALIZING
                    self._aruco_candidates = []
                    self.logger.warn("Robot still detected, waiting for coherent ArUco poses")
            else:
                self._still_since = None

    def on_aruco_pose(self, x: float, y: float, theta_deg: float) -> None:
        if not self.in_progress or self.status != self.STATUS_LOCALIZING:
            return

        pose = (x, y, theta_deg)
        if not self._aruco_candidates:
            self._aruco_candidates.append(pose)
            return

        prev = self._aruco_candidates[-1]
        pos_delta = sqrt((pose[0] - prev[0]) ** 2 + (pose[1] - prev[1]) ** 2)
        angle_delta = self._angle_delta_deg(pose[2], prev[2])

        if pos_delta <= self.aruco_pos_threshold_m and angle_delta <= self.aruco_angle_threshold_deg:
            self._aruco_candidates.append(pose)
        else:
            self.logger.warn(
                f"Auto-placement ArUco coherence reset: dpos={pos_delta:.3f}m dtheta={angle_delta:.1f}deg"
            )
            self._aruco_candidates = [pose]

        if len(self._aruco_candidates) > self.required_aruco_poses:
            self._aruco_candidates.pop(0)

        if len(self._aruco_candidates) >= self.required_aruco_poses:
            selected_pose = self._aruco_candidates[-1]
            self.status = self.STATUS_SETTING_POSE
            self.logger.warn(
                f"ArUco localization locked at ({selected_pose[0]:.3f}, {selected_pose[1]:.3f}, {selected_pose[2]:.1f}deg)"
            )
            self._request_set_pose([selected_pose[0], selected_pose[1], selected_pose[2]])

    def on_set_pose_done(self, success: bool, error_msg: str = "") -> None:
        if not self.in_progress or self.status != self.STATUS_SETTING_POSE:
            return

        if not success:
            self._fail(f"set_pose service call failed: {error_msg}")
            return

        if self._pending_init_pose is None:
            self._fail("missing init pose for move_to")
            return

        self.status = self.STATUS_MOVING_TO_INIT
        self.logger.warn(
            f"Auto-placement move_to init pose ({self._pending_init_pose[0]:.2f}, {self._pending_init_pose[1]:.2f}, {self._pending_init_pose[2]:.1f}deg)"
        )
        self._request_move_to_init(self._pending_init_pose)

    def on_move_result(self, success: bool, error_msg: str = "") -> None:
        if not self.in_progress or self.status != self.STATUS_MOVING_TO_INIT:
            return

        if not success:
            self._fail(f"navigation failed: {error_msg}")
            return

        self.in_progress = False
        self.status = self.STATUS_IDLE
        self.logger.warn("Auto-placement completed, waiting for tirette")
        self._on_completed()

    def ui_state(self) -> str:
        if not self.in_progress:
            return "idle"

        mapping = {
            self.STATUS_WAITING_STILL: "auto_placement_wait_still",
            self.STATUS_LOCALIZING: "auto_placement_localizing",
            self.STATUS_SETTING_POSE: "auto_placement_setting_pose",
            self.STATUS_MOVING_TO_INIT: "auto_placement_moving",
            self.STATUS_FAILED: "auto_placement_failed",
        }
        return mapping.get(self.status, "auto_placement_running")

    def _is_robot_at_rest(self, linear_velocity: float, angular_velocity: float) -> bool:
        return (
            linear_velocity < self.linear_vel_threshold
            and angular_velocity < self.angular_vel_threshold
        )

    def _angle_delta_deg(self, a: float, b: float) -> float:
        diff = (a - b + 180.0) % 360.0 - 180.0
        return abs(diff)

    def _fail(self, reason: str) -> None:
        self.status = self.STATUS_FAILED
        # Keep in_progress=True so UI can keep showing failed state
        self.in_progress = True
        self.logger.error(f"Auto-placement failed: {reason}")
