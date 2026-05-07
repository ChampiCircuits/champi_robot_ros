#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, Twist

from champi_interfaces.action import Navigate
from champi_interfaces.msg import CtrlGoal

import time
from threading import Lock
import math
import diagnostic_msgs.msg
import diagnostic_updater
import yaml
from ament_index_python.packages import get_package_share_directory

from champi_navigation.planning_feedback import ComputePathResult, get_feedback_msg
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap, ObstaclePolygon
from champi_libraries_py.utils.diagnostics import ExecTimeMeasurer
from champi_libraries_py.utils.timeout import Timeout
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.angles import get_yaw
from champi_libraries_py.marker_helper.canva import Canva, items, presets
import champi_navigation.goal_checker as goal_checker


class PlanningDiagnostic:
    """Tracks planning call statistics and produces ROS diagnostics."""

    def __init__(self):
        self._last_ms = None
        self._worst_ms = 0.0
        self._total_ms = 0.0
        self._n_calls = 0
        self._n_failed = 0

    def record(self, planning_ms: float, success: bool):
        self._last_ms = planning_ms
        self._n_calls += 1
        self._total_ms += planning_ms
        if planning_ms > self._worst_ms:
            self._worst_ms = planning_ms
        if not success:
            self._n_failed += 1

    def produce_diagnostics(self, stat):
        if self._last_ms is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'No planning call yet')
            return stat
        avg_ms = self._total_ms / self._n_calls if self._n_calls else 0.0
        fail_rate = self._n_failed / self._n_calls if self._n_calls else 0.0
        level = diagnostic_msgs.msg.DiagnosticStatus.OK
        if self._worst_ms > 75.0 or fail_rate > 0.2:
            level = diagnostic_msgs.msg.DiagnosticStatus.WARN
        stat.summary(level, f'last={self._last_ms:.1f}ms  avg={avg_ms:.1f}ms  worst={self._worst_ms:.1f}ms')
        stat.add('Last planning time (ms)', f'{self._last_ms:.2f}')
        stat.add('Avg planning time (ms)',  f'{avg_ms:.2f}')
        stat.add('Worst planning time (ms)', f'{self._worst_ms:.2f}')
        stat.add('Total calls', str(self._n_calls))
        stat.add('Failed calls (no path)', str(self._n_failed))
        stat.add('Failure rate', f'{fail_rate*100:.1f}%')
        return stat


class PlannerNode(Node):
    """
    Path planner node. It has an action server. It receives Navigate goals and computes a path to reach them.
    Then it publishes the command to the controller node as a CtrlGoal.
    """

    def __init__(self):
        super().__init__('planner_node')

        # Parameters
        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_tolerance = self.declare_parameter('waypoint_tolerance', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_speed_linear = self.declare_parameter('waypoint_speed_linear', rclpy.Parameter.Type.DOUBLE).value
        self.debug = self.declare_parameter('debug', rclpy.Parameter.Type.BOOL).value
        self.robot_radius = self.declare_parameter('robot_radius', rclpy.Parameter.Type.DOUBLE).value
        self.enemy_robot_radius = self.declare_parameter('enemy_robot_radius', rclpy.Parameter.Type.DOUBLE).value
        self.table_width = self.declare_parameter('table_width', rclpy.Parameter.Type.DOUBLE).value
        self.table_height = self.declare_parameter('table_height', rclpy.Parameter.Type.DOUBLE).value
        self.forbidden_area_wait_time = self.declare_parameter('forbidden_area_wait_time', rclpy.Parameter.Type.DOUBLE).value
        self.forbidden_area_margin = self.declare_parameter('forbidden_area_margin', rclpy.Parameter.Type.DOUBLE).value
        self.world_state_file = self.declare_parameter('world_state_file', rclpy.Parameter.Type.STRING).value

        # Print parameters
        self.get_logger().info('Path Planner started with the following parameters:')
        self.get_logger().info(f'loop_period: {self.loop_period}')
        self.get_logger().info(f'waypoint_tolerance: {self.waypoint_tolerance}')
        self.get_logger().info(f'robot_radius: {self.robot_radius}')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.enemy_odom_sub = self.create_subscription(Odometry, '/enemy_pose', self.enemy_odom_callback, 10)
        self.latest_enemy_pose: Pose2D = None

        # Publisher
        self.champi_path_pub = self.create_publisher(CtrlGoal, '/ctrl_goal', 10)
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)
        self.cmd_vel_stop_pub = self.create_publisher(Twist, '/emergency/cmd_vel_stop', 10)

        # Action Server /navigate
        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.navigate_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Path Planner started /navigate server')

        # Timeout object to abort the Navigate goal if it takes too long
        self.timeout = Timeout()

        # Diagnostic updater
        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')
        self.exec_time_measurer = ExecTimeMeasurer()
        self.planning_diagnostic = PlanningDiagnostic()
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)
        updater.add('Timeout', self.timeout.produce_diagnostics)
        updater.add('Path planner', self.planning_diagnostic.produce_diagnostics)

        # Data retrieved from topics
        self.robot_pose: Pose2D = None

        # Current goal handle
        self.goal_handle_navigate = None
        self.planning = False
        self._goal_preempted = False
        self.mutex_exec = Lock()

        # Visibility planner
        self.visibility_planner = VisibilityRoadMap(expand_distance=self.robot_radius)

        # Separate planner with smaller expansion for forbidden area detection
        # This avoids false triggers when the robot is just barely on the C-space boundary
        self.forbidden_area_checker = VisibilityRoadMap(expand_distance=max(0.0, self.robot_radius - self.forbidden_area_margin))

        # Static obstacles: table borders as 4 thin edge polygons
        self.static_obstacles = self._build_border_obstacles()

        # Zone obstacles (loaded from world state file)
        self.zones = self._load_zones()
        # Zone occupancy: all occupied by default (will be updated via topic later)
        self.zone_occupied = {zone['id']: True for zone in self.zones}
        self.get_logger().info(f'Loaded {len(self.zones)} zones as obstacles: {[z["id"] for z in self.zones]}')

        # Visualization
        Canva(self, enable=self.debug)
        self.viz_timer = self.create_timer(0.2, self.viz_timer_callback)

    # ==================================== Obstacle Management ==========================================

    def _load_zones(self):
        """Load zones and elements from the world state YAML file."""
        config_path = get_package_share_directory('champi_brain') + '/config/' + self.world_state_file
        with open(config_path, 'r') as f:
            world_state = yaml.safe_load(f)
        zones = world_state.get('zones', [])
        self.elements = world_state.get('elements', [])
        self.get_logger().info(f'World state loaded from {config_path}: {len(zones)} zones, {len(self.elements)} elements')
        return zones

    def _build_element_obstacles(self):
        """Build obstacle polygons for all nut_box elements (rotated rectangles)."""
        # Nut box dimensions: 150 x 100 mm
        NUT_BOX_W = 0.15  # half-width along local X
        NUT_BOX_H = 0.20  # half-height along local Y
        hw = NUT_BOX_W / 2.0
        hh = NUT_BOX_H / 2.0
        obstacles = []
        for elem in self.elements:
            if elem.get('type') != 'nut_box':
                continue
            cx, cy = elem['x'], elem['y']
            theta = math.radians(elem.get('theta_deg', 0.0) + 90.0)  # world frame theta, add 90deg to convert from element's local frame
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            # 4 corners in local frame, rotated into world frame
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            xs = [cx + cos_t * lx - sin_t * ly for lx, ly in corners]
            ys = [cy + sin_t * lx + cos_t * ly for lx, ly in corners]
            obstacles.append(ObstaclePolygon(xs, ys))
        return obstacles

    def _build_zone_obstacles(self):
        """Build obstacle polygons for all currently occupied zones.

        Zones of type 'secure_zone' are not added as obstacles — the robot is
        allowed to drive through them.
        """
        obstacles = []
        for zone in self.zones:
            if zone.get('type') == 'secure_zone':
                continue
            if not self.zone_occupied.get(zone['id'], False):
                continue
            x = zone['x']
            y = zone['y']
            w = zone['width']
            h = zone['height']
            # Zone position is bottom-left corner
            obstacles.append(ObstaclePolygon(
                [x, x + w, x + w, x],
                [y, y, y + h, y + h]
            ))
        return obstacles

    def _build_border_obstacles(self):
        """Build border obstacles as 4 thin rectangles along the edges of the table."""
        w = self.table_width
        h = self.table_height
        t = 0.01  # thin wall thickness

        borders = [
            # Bottom wall
            ObstaclePolygon([0.0, w, w, 0.0], [0.0, 0.0, -t, -t]),
            # Top wall
            ObstaclePolygon([0.0, w, w, 0.0], [h, h, h + t, h + t]),
            # Left wall
            ObstaclePolygon([0.0, 0.0, -t, -t], [0.0, h, h, 0.0]),
            # Right wall
            ObstaclePolygon([w, w, w + t, w + t], [0.0, h, h, 0.0]),
        ]
        return borders

    def _build_enemy_obstacle(self, enemy_pose: Pose2D):
        """Build a square obstacle around the enemy position."""
        r = self.enemy_robot_radius
        cx, cy = enemy_pose.x, enemy_pose.y
        return ObstaclePolygon(
            [cx - r, cx + r, cx + r, cx - r],
            [cy - r, cy - r, cy + r, cy + r]
        )

    def _get_all_obstacles(self):
        """Get all obstacles: static borders + zones + elements + dynamic enemy."""
        obstacles = list(self.static_obstacles)
        obstacles.extend(self._build_zone_obstacles())
        obstacles.extend(self._build_element_obstacles())
        if self.latest_enemy_pose is not None:
            obstacles.append(self._build_enemy_obstacle(self.latest_enemy_pose))
        return obstacles

    def _filter_outside_table(self, obstacles):
        """Remove obstacles whose bounding box does not overlap the table area.

        Uses a margin equal to robot_radius so C-space expansion near the border
        is never accidentally discarded.
        """
        margin = self.robot_radius
        filtered = []
        for obs in obstacles:
            xs = obs.x_list[:-1]  # skip closing duplicate vertex
            ys = obs.y_list[:-1]
            if (max(xs) >= -margin and min(xs) <= self.table_width  + margin and
                    max(ys) >= -margin and min(ys) <= self.table_height + margin):
                filtered.append(obs)
        return filtered

    def _get_static_obstacles(self):
        """Static obstacles: table borders + zones + nut_box elements (cached by the planner)."""
        obstacles = list(self.static_obstacles)
        obstacles.extend(self._build_zone_obstacles())
        obstacles.extend(self._build_element_obstacles())
        return self._filter_outside_table(obstacles)

    def _get_dynamic_obstacles(self):
        """Dynamic obstacles: enemy robot (changes every call)."""
        if self.latest_enemy_pose is not None:
            return self._filter_outside_table(
                [self._build_enemy_obstacle(self.latest_enemy_pose)]
            )
        return []

    def _is_robot_in_forbidden_area(self):
        """Check if the robot is inside any expanded obstacle polygon (with margin).
        
        Uses a smaller expansion than path planning so the robot must be significantly
        inside the C-space boundary before triggering the emergency stop.
        """
        if self.robot_pose is None:
            return False
        obstacles = self._get_all_obstacles()
        expanded_obstacles = self.forbidden_area_checker.build_expanded_obstacles(obstacles)
        for obs in expanded_obstacles:
            if VisibilityRoadMap._point_in_polygon(self.robot_pose.x, self.robot_pose.y, obs):
                return True
        return False

    def _find_nearest_exit_point(self):
        """Find the nearest point outside all expanded obstacles by projecting onto polygon edges."""
        obstacles = self._get_all_obstacles()
        expanded_obstacles = self.forbidden_area_checker.build_expanded_obstacles(obstacles)

        best_point = None
        best_dist = float('inf')

        for obs in expanded_obstacles:
            if not VisibilityRoadMap._point_in_polygon(self.robot_pose.x, self.robot_pose.y, obs):
                continue
            # Find closest point on polygon boundary and move slightly outside
            for i in range(len(obs.x_list) - 1):
                ax, ay = obs.x_list[i], obs.y_list[i]
                bx, by = obs.x_list[i + 1], obs.y_list[i + 1]
                # Project robot position onto edge segment
                dx, dy = bx - ax, by - ay
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-9:
                    continue
                t = max(0.0, min(1.0, ((self.robot_pose.x - ax) * dx + (self.robot_pose.y - ay) * dy) / seg_len_sq))
                proj_x = ax + t * dx
                proj_y = ay + t * dy
                # Move slightly outward (away from polygon center)
                nx, ny = -dy, dx  # normal to edge
                norm = (nx * nx + ny * ny) ** 0.5
                if norm < 1e-9:
                    continue
                nx, ny = nx / norm, ny / norm
                # Choose direction pointing away from polygon interior
                cx = sum(obs.x_list[:-1]) / (len(obs.x_list) - 1)
                cy = sum(obs.y_list[:-1]) / (len(obs.y_list) - 1)
                if nx * (proj_x - cx) + ny * (proj_y - cy) < 0:
                    nx, ny = -nx, -ny
                exit_x = proj_x + nx * 0.03  # 3cm outside
                exit_y = proj_y + ny * 0.03
                dist = ((self.robot_pose.x - exit_x) ** 2 + (self.robot_pose.y - exit_y) ** 2) ** 0.5
                if dist < best_dist:
                    best_dist = dist
                    best_point = Pose2D(x=exit_x, y=exit_y, theta=self.robot_pose.theta)

        return best_point

    # ==================================== Path Planning ==========================================

    def compute_path(self, start: Pose2D, goal: Pose2D):
        """Compute path using visibility road map planner.

        Returns:
            list[Pose2D] or None: list of waypoints from start to goal, or None if no path found.
        """
        static_obstacles  = self._get_static_obstacles()
        dynamic_obstacles = self._get_dynamic_obstacles()

        self.get_logger().debug(f'Computing path from ({start.x:.2f}, {start.y:.2f}) to ({goal.x:.2f}, {goal.y:.2f}) '
                                f'with {len(static_obstacles)} static + {len(dynamic_obstacles)} dynamic obstacles')

        rx, ry, planning_ms = self.visibility_planner.planning(
            start.x, start.y, goal.x, goal.y, static_obstacles, dynamic_obstacles
        )

        if not rx or not ry:
            self.planning_diagnostic.record(planning_ms, success=False)
            self.get_logger().warn(f'No path found from ({start.x:.2f}, {start.y:.2f}) to ({goal.x:.2f}, {goal.y:.2f}) '
                                   f'in {planning_ms:.1f}ms')
            return None

        self.planning_diagnostic.record(planning_ms, success=True)
        self.get_logger().info(f'Path found with {len(rx)} waypoints in {planning_ms:.1f}ms')

        # rx, ry are from goal to start, reverse them
        rx.reverse()
        ry.reverse()

        # Convert to Pose2D waypoints (use goal theta for all)
        waypoints = [Pose2D(x=x, y=y, theta=goal.theta) for x, y in zip(rx, ry)]

        return waypoints

    # ==================================== ROS2 topics Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = Pose2D(pose=msg.pose.pose)

    def enemy_odom_callback(self, msg):
        self.latest_enemy_pose = Pose2D(pose=msg.pose.pose)
        self.get_logger().info(f'Enemy pose received: ({self.latest_enemy_pose.x:.2f}, {self.latest_enemy_pose.y:.2f})', throttle_duration_sec=2.)

    # ==================================== Action Server Callbacks ==========================================

    def navigate_callback(self, navigate_goal: Navigate.Goal):
        self.get_logger().info(f'[NAV] navigate_callback: New Navigate request received to pose: '
                                f'({navigate_goal.pose.position.x:.3f}, {navigate_goal.pose.position.y:.3f}, {get_yaw(navigate_goal.pose)*180.0/3.14159:.1f}deg), '
                                f'timeout={navigate_goal.timeout:.1f}s, currently_planning={self.planning}')

        self.current_navigate_goal = navigate_goal

        if self.planning:
            self.get_logger().warn(f'[NAV] navigate_callback: Preempting previous goal for new one!')
            self._goal_preempted = True  # signals execute_callback to exit its loop cleanly

        self.planning = True
        self.get_logger().debug(f'[NAV] navigate_callback: Goal ACCEPTED')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().debug(f'[NAV] cancel_callback: Cancel requested, currently_planning={self.planning}')
        if self.planning:
            self._goal_preempted = True  # signals execute_callback to exit its loop cleanly
            self.get_logger().info('[NAV] cancel_callback: Cancel accepted, signalling execute_callback to stop')
        else:
            self.get_logger().warn('[NAV] cancel_callback: No active goal to cancel!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().debug(f'[NAV] execute_callback: Waiting to acquire mutex (blocked={self.mutex_exec.locked()})')
        self.mutex_exec.acquire()
        self.get_logger().debug(f'[NAV] execute_callback: Mutex acquired, goal_active={goal_handle.is_active}')
        self.goal_handle_navigate = goal_handle
        self._goal_preempted = False  # reset for this execution

        # =================================== INITIALIZATION ==========================================

        while rclpy.ok() and self.robot_pose is None and goal_handle.is_active and not self._goal_preempted:
            self.get_logger().info(f'[NAV] execute_callback: Waiting for init messages, robot_pose={self.robot_pose is not None}', throttle_duration_sec=1.)
            feedback_msg = get_feedback_msg(ComputePathResult.INITIALIZING, [], 0)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(self.loop_period)

        # =================================== MAIN LOOP ==========================================

        navigate_goal_reached = False
        self.timeout.start(self.current_navigate_goal.timeout)

        goal_pose = Pose2D(pose=self.current_navigate_goal.pose)
        self.get_logger().debug(f'[NAV] execute_callback: Starting main loop, goal=({goal_pose.x:.3f}, {goal_pose.y:.3f}, {goal_pose.theta:.2f}rad), '
                               f'robot=({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f}), timeout={self.current_navigate_goal.timeout:.1f}s')

        # Compute initial path
        waypoints = self.compute_path(self.robot_pose, goal_pose)
        current_waypoint_idx = 1  # Skip waypoint 0 (always robot's current position)
        self.get_logger().debug(f'[NAV] execute_callback: Initial path computed, waypoints={len(waypoints) if waypoints else 0}')

        while rclpy.ok() and goal_handle.is_active and not navigate_goal_reached and not self._goal_preempted:

            self.exec_time_measurer.start()
            t_loop_start = time.time()

            # Check if the timeout is reached
            if self.timeout.is_elapsed():
                self.get_logger().warn(f'[NAV] execute_callback: TIMEOUT reached! Aborting goal.')
                goal_handle.abort(Navigate.Result(success=False, message='Timeout!'))
                self.timeout.reset()
                self.exec_time_measurer.stop()
                continue

            # Check if robot is inside a forbidden (expanded obstacle) area
            if self._is_robot_in_forbidden_area():
                self.get_logger().warn(f'[NAV] execute_callback: Robot is inside FORBIDDEN AREA at ({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f})! Stopping...')
                self.publish_stop()
                time.sleep(self.forbidden_area_wait_time)

                # Find exit point and navigate to it
                exit_point = self._find_nearest_exit_point()
                if exit_point is not None:
                    ctrl_goal = self.create_ctrl_goal_from_navigate_goal(self.current_navigate_goal, is_waypoint=True)
                    ctrl_goal.pose = exit_point.to_ros_pose()
                    self.champi_path_pub.publish(ctrl_goal)
                    # Wait until robot exits the forbidden area
                    while rclpy.ok() and goal_handle.is_active and self._is_robot_in_forbidden_area():
                        time.sleep(self.loop_period)
                self.exec_time_measurer.stop()
                continue

            # Recompute path (handles dynamic enemy obstacle)
            new_waypoints = self.compute_path(self.robot_pose, goal_pose)
            if new_waypoints is not None:
                waypoints = new_waypoints
                current_waypoint_idx = 1  # Skip waypoint 0 (always robot's current position)

            # Determine result for feedback
            if waypoints is None or len(waypoints) < 2:
                result = ComputePathResult.NO_PATH_FOUND
                # No valid path - publish feedback and wait
                feedback_msg = get_feedback_msg(result, [], self.current_navigate_goal.max_linear_speed)
                goal_handle.publish_feedback(feedback_msg)
                self.exec_time_measurer.stop()
                sleep_time = max(0, self.loop_period - (time.time() - t_loop_start))
                time.sleep(sleep_time)
                continue

            if len(waypoints) == 2:
                result = ComputePathResult.SUCCESS_STRAIGHT
            else:
                result = ComputePathResult.SUCCESS_AVOIDANCE

            # Current target waypoint
            target_wp = waypoints[current_waypoint_idx]

            # Check if current waypoint is reached
            is_last_waypoint = (current_waypoint_idx >= len(waypoints) - 1)
            if goal_checker.is_goal_reached(target_wp,
                                            self.robot_pose,
                                            self.current_navigate_goal.end_speed == 0 and is_last_waypoint,
                                            self.current_navigate_goal.do_look_at_point and is_last_waypoint,
                                            Pose2D(point=self.current_navigate_goal.look_at_point) if is_last_waypoint else target_wp,
                                            self.current_navigate_goal.robot_angle_when_looking_at_point if is_last_waypoint else 0.0,
                                            self.current_navigate_goal.linear_tolerance if is_last_waypoint else self.waypoint_tolerance,
                                            self.current_navigate_goal.angular_tolerance if is_last_waypoint else 3.14):
                if is_last_waypoint:
                    self.get_logger().info(f'[NAV] execute_callback: FINAL GOAL REACHED at robot=({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f}), '
                                          f'target=({target_wp.x:.3f}, {target_wp.y:.3f})')
                    navigate_goal_reached = True
                    self.exec_time_measurer.stop()
                    continue
                else:
                    self.get_logger().debug(f'[NAV] execute_callback: Waypoint {current_waypoint_idx}/{len(waypoints)-1} reached at '
                                           f'({target_wp.x:.3f}, {target_wp.y:.3f}), advancing to next')
                    current_waypoint_idx += 1
                    target_wp = waypoints[current_waypoint_idx]

            # Create and publish CtrlGoal
            is_last_waypoint = (current_waypoint_idx >= len(waypoints) - 1)
            is_waypoint = not is_last_waypoint
            ctrl_goal = self.create_ctrl_goal_from_navigate_goal(self.current_navigate_goal, is_waypoint=is_waypoint)
            ctrl_goal.pose = target_wp.to_ros_pose()
            self.champi_path_pub.publish(ctrl_goal)

            # Publish action feedback
            remaining_path = [self.robot_pose] + waypoints[current_waypoint_idx:]
            feedback_msg = get_feedback_msg(result, remaining_path, self.current_navigate_goal.max_linear_speed)
            goal_handle.publish_feedback(feedback_msg)

            # Publish path for visualization
            self.publish_path([p.to_ros_pose() for p in remaining_path])

            # Draw obstacles and path with marker helper
            self.draw_viz(waypoints, current_waypoint_idx)

            self.exec_time_measurer.stop()

            sleep_time = max(0, self.loop_period - (time.time() - t_loop_start))
            time.sleep(sleep_time)

        self.timeout.reset()

        # ============================ FILL ACTION RESULT ====================================

        self.publish_stop()
        self.publish_path([])

        if navigate_goal_reached:
            # Robot reached the goal — succeed (or honor a simultaneous cancel cleanly)
            result = Navigate.Result(success=True, message='Goal reached!')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled(result)
            else:
                goal_handle.succeed(result)
            self.get_logger().info(f'[NAV] execute_callback: RESULT => Goal REACHED at ({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f})')
        elif self._goal_preempted:
            # Preempted by a new navigate goal or a cancel request
            result = Navigate.Result(success=False, message='Goal aborted!')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled(result)
            elif goal_handle.is_active:
                goal_handle.abort(result)
            self.get_logger().info('[NAV] execute_callback: RESULT => Goal preempted (new goal or cancel)')
        elif not goal_handle.is_active:
            # Aborted internally (e.g. timeout) — abort() already sent result from within the loop
            self.get_logger().info('[NAV] execute_callback: RESULT => Goal aborted (timeout or forbidden area)')
            result = Navigate.Result(success=False, message='Goal aborted!')
        elif not rclpy.ok():
            result = Navigate.Result(success=False, message='Node shutdown!')
            goal_handle.abort(result)
            self.get_logger().info('[NAV] execute_callback: RESULT => Node shutdown')
        else:
            result = Navigate.Result(success=False, message='Unknown error!')
            goal_handle.abort(result)
            self.get_logger().error('[NAV] execute_callback: RESULT => Unknown exit state!')

        # Set planning=False last. If a new goal arrived while we were in the result section
        # (navigate_callback set planning=True again) we must NOT overwrite it.
        if not self._goal_preempted:
            self.planning = False

        self.get_logger().debug(f'[NAV] execute_callback: Releasing mutex')
        self.mutex_exec.release()
        return result

    # ====================================== Visualization ==========================================

    def viz_timer_callback(self):
        """Periodically draw obstacles even when not navigating."""
        if not self.planning:
            self.draw_viz(None, 0)

    def draw_viz(self, waypoints, current_waypoint_idx):
        Canva().clear()

        # Draw obstacles
        obstacles = self._get_all_obstacles()
        self.get_logger().debug(f'Drawing {len(obstacles)} obstacles (enemy_pose={self.latest_enemy_pose is not None})')
        for obs in obstacles:
            points = list(zip(obs.x_list, obs.y_list))
            Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.RED), frame_id='odom')

        # Draw expanded (C-space) obstacles
        expanded_obstacles = self.visibility_planner.build_expanded_obstacles(obstacles)
        for obs in expanded_obstacles:
            points = list(zip(obs.x_list, obs.y_list))
            Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.ORANGE), frame_id='odom')

        # Draw path
        if waypoints and current_waypoint_idx < len(waypoints):
            path_points = [(wp.x, wp.y) for wp in waypoints[current_waypoint_idx:]]
            if len(path_points) >= 2:
                Canva().add(items.Polyline(path_points, size=presets.LINE_MEDIUM, color=presets.GREEN), frame_id='odom')
            # Draw waypoints as spheres
            Canva().add(items.Spheres(path_points, color=presets.CYAN), frame_id='odom')

        Canva().draw()

    # ====================================== Utils ==========================================

    def publish_stop(self):
        ctrl_goal = CtrlGoal()
        self.champi_path_pub.publish(ctrl_goal)

    def publish_path(self, path: list[Pose]):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for pose in path:
            path_msg.poses.append(self.pose_to_pose_stamped(pose, 'odom'))
        self.path_publisher_viz.publish(path_msg)

    def pose_to_pose_stamped(self, pose: Pose, frame_id: str) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = pose
        return pose_stamped

    def create_ctrl_goal_from_navigate_goal(self, navigate_goal: Navigate.Goal, is_waypoint) -> CtrlGoal:
        ctrl_goal = CtrlGoal()
        ctrl_goal.pose = navigate_goal.pose

        if is_waypoint:
            ctrl_goal.end_speed = self.waypoint_speed_linear
            ctrl_goal.linear_tolerance = self.waypoint_tolerance
            ctrl_goal.max_linear_speed = self.waypoint_speed_linear
        else:
            ctrl_goal.end_speed = navigate_goal.end_speed
            ctrl_goal.linear_tolerance = navigate_goal.linear_tolerance
            ctrl_goal.max_linear_speed = navigate_goal.max_linear_speed

        ctrl_goal.max_angular_speed = navigate_goal.max_angular_speed
        ctrl_goal.accel_linear = navigate_goal.accel_linear
        ctrl_goal.accel_angular = navigate_goal.accel_angular
        ctrl_goal.angular_tolerance = navigate_goal.angular_tolerance
        ctrl_goal.do_look_at_point = navigate_goal.do_look_at_point
        ctrl_goal.look_at_point = navigate_goal.look_at_point
        ctrl_goal.robot_angle_when_looking_at_point = navigate_goal.robot_angle_when_looking_at_point

        return ctrl_goal


    # ====================================== Main ==========================================


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
