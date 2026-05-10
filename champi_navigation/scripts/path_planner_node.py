#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped

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
from champi_navigation.planner import Planner, PlannerStatus
from champi_navigation.planning_feedback import ComputePathResult, get_feedback_msg
from champi_navigation.pose_controller_manager import PoseControllerManager
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap
from champi_libraries_py.utils.diagnostics import ExecTimeMeasurer
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.angles import get_yaw
from champi_libraries_py.marker_helper.canva import Canva, items, presets


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
        # (now managed internally by Planner)

        # Diagnostic updater
        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')
        self.exec_time_measurer = ExecTimeMeasurer()
        self.pose_controller_manager = PoseControllerManager(
            node=self,
            waypoint_speed_linear=self.waypoint_speed_linear,
            waypoint_tolerance=self.waypoint_tolerance
        )
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)
        updater.add('Path planner', self._produce_planning_diagnostics)

        # Data retrieved from topics
        self.robot_pose: Pose2D = None

        # Current goal handle
        self.goal_handle_navigate = None
        self.planning = False
        self.new_goal_waiting = False
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

        # Planner (ROS-free step-based state machine)
        self.planner = Planner(
            obstacle_manager=self.obstacle_manager,
            visibility_planner=self.visibility_planner,
            waypoint_tolerance=self.waypoint_tolerance,
            waypoint_speed_linear=self.waypoint_speed_linear,
            forbidden_area_wait_time=self.forbidden_area_wait_time,
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
        changed = self.planner.update_obstacle_states(states)
        if changed:
            self.get_logger().info('[NAV] Obstacle states updated, planner cache invalidated')

    # ==================================== ROS2 topics Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = Pose2D(pose=msg.pose.pose)

    def enemy_odom_callback(self, msg):
        pose = Pose2D(pose=msg.pose.pose)
        self.planner.set_enemy_pose(pose.x, pose.y)
        self.get_logger().info(f'Enemy pose received: ({pose.x:.2f}, {pose.y:.2f})', throttle_duration_sec=2.)

    # ==================================== Action Server Callbacks ==========================================

    def navigate_callback(self, navigate_goal: Navigate.Goal):
        self.get_logger().info(f'[NAV] navigate_callback: New Navigate request received to pose: '
                                f'({navigate_goal.pose.position.x:.3f}, {navigate_goal.pose.position.y:.3f}, {get_yaw(navigate_goal.pose)*180.0/3.14159:.1f}deg), '
                                f'timeout={navigate_goal.timeout:.1f}s, currently_planning={self.planning}')

        self.current_navigate_goal = navigate_goal

        if self.planning:
            self.get_logger().warn(f'[NAV] navigate_callback: Preempting previous goal for new one!')
            self.planner.cancel()
            self.new_goal_waiting = True  # signals execute_callback to exit its loop cleanly

        self.planning = True
        self.get_logger().debug(f'[NAV] navigate_callback: Goal ACCEPTED')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().debug(f'[NAV] cancel_callback: Cancel requested, currently_planning={self.planning}')
        if self.planning:
            self.planner.cancel()
            self.new_goal_waiting = True  # signals execute_callback to exit its loop cleanly
            self.get_logger().info('[NAV] cancel_callback: Cancel accepted, signalling execute_callback to stop')
        else:
            self.get_logger().warn('[NAV] cancel_callback: No active goal to cancel!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().debug(f'[NAV] execute_callback: Waiting to acquire mutex (blocked={self.mutex_exec.locked()})')
        self.mutex_exec.acquire()
        self.get_logger().debug(f'[NAV] execute_callback: Mutex acquired, goal_active={goal_handle.is_active}')
        self.goal_handle_navigate = goal_handle
        self.new_goal_waiting = False  # reset for this execution

        # =================================== START PLANNER ==========================================

        self.planner.start(self.current_navigate_goal)

        # =================================== MAIN LOOP ==========================================

        navigate_goal_reached = False

        while rclpy.ok() and goal_handle.is_active and not navigate_goal_reached and not self.new_goal_waiting:

            self.exec_time_measurer.start()
            t_loop_start = time.time()

            output = self.planner.step(self.robot_pose)

            if output.status == PlannerStatus.INITIALIZING:
                self.get_logger().info(
                    f'[NAV] execute_callback: Waiting for robot pose', throttle_duration_sec=1.
                )
                goal_handle.publish_feedback(
                    get_feedback_msg(ComputePathResult.INITIALIZING, [], 0)
                )

            elif output.status == PlannerStatus.TIMED_OUT:
                self.get_logger().warn(f'[NAV] execute_callback: TIMEOUT reached! Aborting goal.')
                if output.send_stop:
                    self.pose_controller_manager.publish_stop()
                goal_handle.abort(Navigate.Result(success=False, message='Timeout!'))
                # goal_handle.is_active is now False; loop exits on next iteration check

            elif output.status == PlannerStatus.IN_FORBIDDEN_AREA:
                if output.send_stop:
                    self.pose_controller_manager.publish_stop()
                elif output.ctrl_goal is not None:
                    self.pose_controller_manager.publish_ctrl_goal(
                        output.ctrl_goal,
                        metadata=self.current_navigate_goal,
                        is_waypoint=output.is_waypoint,
                    )

            elif output.status == PlannerStatus.NO_PATH:
                feedback_msg = get_feedback_msg(output.path_result, [], output.max_linear_speed)
                goal_handle.publish_feedback(feedback_msg)

            elif output.status == PlannerStatus.RUNNING:
                self.pose_controller_manager.publish_ctrl_goal(
                    output.ctrl_goal,
                    metadata=self.current_navigate_goal,
                    is_waypoint=output.is_waypoint,
                )
                feedback_msg = get_feedback_msg(
                    output.path_result, output.remaining_path, output.max_linear_speed
                )
                goal_handle.publish_feedback(feedback_msg)
                self.publish_path([p.to_ros_pose() for p in output.remaining_path])
                self.draw_viz(output.waypoints, output.waypoint_idx)

            elif output.status == PlannerStatus.GOAL_REACHED:
                self.get_logger().info(
                    f'[NAV] execute_callback: FINAL GOAL REACHED at robot=({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f})'
                )
                navigate_goal_reached = True

            self.exec_time_measurer.stop()

            sleep_time = max(0, self.loop_period - (time.time() - t_loop_start))
            time.sleep(sleep_time)

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
        elif self.new_goal_waiting:
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
        if not self.new_goal_waiting:
            self.planning = False

        self.get_logger().debug(f'[NAV] execute_callback: Releasing mutex')
        self.mutex_exec.release()
        return result

    # ====================================== Visualization ==========================================

    def viz_timer_callback(self):
        """Periodically draw obstacles even when not navigating."""
        if not self.planning:
            self.draw_viz(None, 0)


    def _produce_planning_diagnostics(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        metrics = self.planner.planning_metrics
        
        if metrics.last_ms is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "No planning call yet")
            return stat

        level = diagnostic_msgs.msg.DiagnosticStatus.OK
        if metrics.worst_ms > 75.0 or metrics.fail_rate > 0.2:
            level = diagnostic_msgs.msg.DiagnosticStatus.WARN

        stat.summary(
            level,
            f"last={metrics.last_ms:.1f}ms  avg={metrics.avg_ms:.1f}ms  worst={metrics.worst_ms:.1f}ms",
        )
        stat.add("Last planning time (ms)", f"{metrics.last_ms:.2f}")
        stat.add("Avg planning time (ms)", f"{metrics.avg_ms:.2f}")
        stat.add("Worst planning time (ms)", f"{metrics.worst_ms:.2f}")
        stat.add("Total calls", str(metrics.n_calls))
        stat.add("Failed calls (no path)", str(metrics.n_failed))
        stat.add("Failure rate", f"{metrics.fail_rate * 100:.1f}%")
        
        return stat

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


    def draw_viz(self, waypoints, current_waypoint_idx):
            Canva().clear()

            geom = self.planner.get_debug_geometry()

            self.get_logger().debug(f'Drawing {len(geom.obstacles)} obstacles')

            for points in geom.obstacles:
                Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.RED), frame_id='odom')

            for points in geom.expanded_obstacles:
                Canva().add(items.Polyline(points, size=presets.LINE_THIN, color=presets.ORANGE), frame_id='odom')

            if waypoints and current_waypoint_idx < len(waypoints):
                path_points = [(wp.x, wp.y) for wp in waypoints[current_waypoint_idx:]]
                if len(path_points) >= 2:
                    Canva().add(items.Polyline(path_points, size=presets.LINE_MEDIUM, color=presets.GREEN), frame_id='odom')
                # Draw waypoints as spheres
                Canva().add(items.Spheres(path_points, color=presets.CYAN), frame_id='odom')

            Canva().draw()


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
