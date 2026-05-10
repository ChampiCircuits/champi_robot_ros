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

import time
from threading import Lock
import json
import diagnostic_msgs.msg
import diagnostic_updater
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile, DurabilityPolicy

from champi_navigation.obstacle_manager import ObstacleManager
from champi_navigation.planning_feedback import ComputePathResult, get_feedback_msg
from champi_navigation.pose_controller_manager import PoseControllerManager
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap
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

        # Publisher
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)

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
        self.pose_controller_manager = PoseControllerManager(
            node=self,
            waypoint_speed_linear=self.waypoint_speed_linear,
            waypoint_tolerance=self.waypoint_tolerance
        )
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

        # Obstacle manager
        config_path = get_package_share_directory('champi_brain') + '/config/' + self.world_state_file
        self.obstacle_manager = ObstacleManager(
            config_path=config_path,
            table_width=self.table_width,
            table_height=self.table_height,
            robot_radius=self.robot_radius,
            enemy_robot_radius=self.enemy_robot_radius,
            forbidden_area_margin=self.forbidden_area_margin,
        )
        self.get_logger().info(
            f'ObstacleManager initialised: {len(self.obstacle_manager._zones)} zones, '
            f'{len(self.obstacle_manager._elements)} elements'
        )

        # Subscribe to brain's obstacle state updates (latched — replayed on connect)
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(StringMsg, '/planner/obstacle_states',
                                 self._obstacle_states_callback, _latched_qos)

        # Visualization
        Canva(self, enable=self.debug)
        self.viz_timer = self.create_timer(0.2, self.viz_timer_callback)

    # ==================================== Obstacle Management ==========================================

    def _obstacle_states_callback(self, msg: StringMsg) -> None:
        """Receive the brain's full obstacle state dict and forward to ObstacleManager."""
        states = json.loads(msg.data)
        changed = self.obstacle_manager.update_states(states)
        if changed:
            self.visibility_planner.invalidate_cache()
            self.get_logger().info('[NAV] Obstacle states updated, planner cache invalidated')

    def _is_robot_in_forbidden_area(self) -> bool:
        if self.robot_pose is None:
            return False
        return self.obstacle_manager.is_point_in_forbidden_area(self.robot_pose.x, self.robot_pose.y)

    def _find_nearest_exit_point(self):
        """Return exit Pose2D (preserving robot theta) or None."""
        result = self.obstacle_manager.find_nearest_exit_point(self.robot_pose.x, self.robot_pose.y)
        if result is None:
            return None
        exit_x, exit_y = result
        return Pose2D(x=exit_x, y=exit_y, theta=self.robot_pose.theta)

    # ==================================== Path Planning ==========================================

    def compute_path(self, start: Pose2D, goal: Pose2D):
        """Compute path using visibility road map planner.

        Returns:
            list[Pose2D] or None: list of waypoints from start to goal, or None if no path found.
        """
        static_obstacles  = self.obstacle_manager.get_static_obstacles()
        dynamic_obstacles = self.obstacle_manager.get_dynamic_obstacles()

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
        pose = Pose2D(pose=msg.pose.pose)
        self.obstacle_manager.set_enemy_pose(pose.x, pose.y)
        self.get_logger().info(f'Enemy pose received: ({pose.x:.2f}, {pose.y:.2f})', throttle_duration_sec=2.)

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
                self.pose_controller_manager.publish_stop()
                time.sleep(self.forbidden_area_wait_time)

                # Find exit point and navigate to it
                exit_point = self._find_nearest_exit_point()
                if exit_point is not None:
                    self.pose_controller_manager.publish_ctrl_goal(exit_point, metadata=self.current_navigate_goal, is_waypoint=True)
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

            # Publish CtrlGoal
            is_last_waypoint = (current_waypoint_idx >= len(waypoints) - 1)
            self.pose_controller_manager.publish_ctrl_goal(target_wp, metadata=self.current_navigate_goal, is_waypoint=not is_last_waypoint)

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

        self.pose_controller_manager.publish_stop()
        self.publish_path([])

        if navigate_goal_reached:
            # Robot reached the goal — succeed (or honor a simultaneous cancel cleanly)
            result = Navigate.Result(success=True, message='Goal reached!')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.succeed()
            self.get_logger().info(f'[NAV] execute_callback: RESULT => Goal REACHED at ({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f})')
        elif self._goal_preempted:
            # Preempted by a new navigate goal or a cancel request
            result = Navigate.Result(success=False, message='Goal aborted!')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().info('[NAV] execute_callback: RESULT => Goal preempted (new goal or cancel)')
        elif not goal_handle.is_active:
            # Aborted internally (e.g. timeout) — abort() already sent result from within the loop
            self.get_logger().info('[NAV] execute_callback: RESULT => Goal aborted (timeout or forbidden area)')
            result = Navigate.Result(success=False, message='Goal aborted!')
        elif not rclpy.ok():
            result = Navigate.Result(success=False, message='Node shutdown!')
            goal_handle.abort()
            self.get_logger().info('[NAV] execute_callback: RESULT => Node shutdown')
        else:
            result = Navigate.Result(success=False, message='Unknown error!')
            goal_handle.abort()
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
        obstacles = self.obstacle_manager.get_all_obstacles()
        self.get_logger().debug(f'Drawing {len(obstacles)} obstacles')
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
