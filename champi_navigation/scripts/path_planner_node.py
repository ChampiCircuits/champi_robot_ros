#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, String as StringMsg
import diagnostic_msgs.msg
import diagnostic_updater
from rclpy.qos import QoSProfile, DurabilityPolicy

from champi_interfaces.action import Navigate
from ament_index_python.packages import get_package_share_directory

import time
import json
import math
from threading import Lock
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum, auto

# Assuming these imports exist in your workspace
from champi_navigation.obstacle_manager import ObstacleManager
from champi_navigation.planner_visibility import PlannerVisibility
from champi_navigation.planner_straight import PlannerStraight
from champi_navigation.planning_feedback import get_feedback_msg
from champi_navigation.pose_controller_manager import PoseControllerManager
from champi_navigation.visibility_planner.visibility_road_map import VisibilityRoadMap
from champi_libraries_py.utils.diagnostics import ExecTimeMeasurer
from champi_libraries_py.data_types.geometry import Pose2D
from champi_libraries_py.utils.angles import get_yaw
from champi_libraries_py.marker_helper.canva import Canva
from champi_navigation.planner_status import PlannerStatus


_INITIALIZING_FEEDBACK_RESULT = getattr(
    Navigate.Feedback,
    'INITIALIZING',
    getattr(Navigate.Feedback, 'INTITIALIZING')
)


class ActionState(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    ABORTED = auto()


@dataclass
class ProcessResult:
    state: ActionState
    feedback: Optional[Navigate.Feedback] = None
    message: str = ""


@dataclass
class PlannerConfig:
    loop_period: float
    waypoint_tolerance: float
    waypoint_speed_linear: float
    enemy_front_max_ahead_distance_m: float
    enemy_front_max_lateral_offset_m: float
    debug: bool
    robot_radius: float
    enemy_robot_radius: float
    table_width: float
    table_height: float
    forbidden_area_wait_time: float
    forbidden_area_margin: float
    world_state_file: str


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.config = self._load_parameters()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.enemy_odom_sub = self.create_subscription(Odometry, '/enemy_pose', self.enemy_odom_callback, 10)
        # self.collision_detector_sub = self.create_subscription(Bool, '/collision_detected', self.collision_detector_callback, 10)
        
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(StringMsg, '/planner/obstacle_states', self._obstacle_states_callback, _latched_qos)

        # Publisher
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)

        # Action Server
        self.action_server_navigate = ActionServer(
            self, Navigate, '/navigate',
            self.execute_callback,
            goal_callback=self.navigate_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Path Planner started /navigate server')

        # Diagnostics & Tracking
        self.exec_time_measurer = ExecTimeMeasurer()
        self._setup_diagnostics()

        self.robot_pose: Optional[Pose2D] = None
        self.current_navigate_goal: Optional[Navigate.Goal] = None
        
        # State Flags & Concurrency Protection
        self.planning: bool = False
        self.new_goal_waiting: bool = False
        self.state_lock = Lock()  # Protects planning, new_goal_waiting, and current_navigate_goal
        self.mutex_exec = Lock()  # Protects the execution loop

        # Core Components
        self.visibility_planner = VisibilityRoadMap(expand_distance=self.config.robot_radius)
        self.obstacle_manager = self._setup_obstacle_manager()
        
        self.planner_avoidance = PlannerVisibility(
            obstacle_manager=self.obstacle_manager,
            visibility_planner=self.visibility_planner,
            waypoint_tolerance=self.config.waypoint_tolerance,
            waypoint_speed_linear=self.config.waypoint_speed_linear,
            forbidden_area_wait_time=self.config.forbidden_area_wait_time,
        )

        self.planner_straight = PlannerStraight(
            waypoint_tolerance=self.config.waypoint_tolerance,
            enemy_front_max_ahead_distance_m=self.config.enemy_front_max_ahead_distance_m,
            enemy_front_max_lateral_offset_m=self.config.enemy_front_max_lateral_offset_m,
        )

        self.active_planner = self.planner_straight

        self.pose_controller_manager = PoseControllerManager(
            node=self,
            waypoint_speed_linear=self.config.waypoint_speed_linear,
            waypoint_tolerance=self.config.waypoint_tolerance
        )

        self.collision_detected: bool = False

        # Visualization
        Canva(self, enable=self.config.debug)
        self.viz_timer = self.create_timer(0.2, self.viz_timer_callback)

    # ==================================== Initialization Helpers ==========================================

    def _load_parameters(self) -> PlannerConfig:
        return PlannerConfig(
            loop_period=self.declare_parameter('planner_loop_period', 0.1).value,
            waypoint_tolerance=self.declare_parameter('waypoint_tolerance', 0.05).value,
            waypoint_speed_linear=self.declare_parameter('waypoint_speed_linear', 0.5).value,
            enemy_front_max_ahead_distance_m=self.declare_parameter('enemy_front_max_ahead_distance_m', 0.6).value,
            enemy_front_max_lateral_offset_m=self.declare_parameter('enemy_front_max_lateral_offset_m', 0.25).value,
            debug=self.declare_parameter('debug', False).value,
            robot_radius=self.declare_parameter('robot_radius', 0.2).value,
            enemy_robot_radius=self.declare_parameter('enemy_robot_radius', 0.2).value,
            table_width=self.declare_parameter('table_width', 3.0).value,
            table_height=self.declare_parameter('table_height', 2.0).value,
            forbidden_area_wait_time=self.declare_parameter('forbidden_area_wait_time', 2.0).value,
            forbidden_area_margin=self.declare_parameter('forbidden_area_margin', 0.05).value,
            world_state_file=self.declare_parameter('world_state_file', 'default.json').value,
        )


    def _setup_diagnostics(self) -> None:
        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)
        updater.add('Path planner', self._produce_planning_diagnostics)

    def _setup_obstacle_manager(self) -> ObstacleManager:
        config_path = f"{get_package_share_directory('champi_brain')}/config/{self.config.world_state_file}"
        manager = ObstacleManager(
            config_path=config_path,
            table_width=self.config.table_width,
            table_height=self.config.table_height,
            robot_radius=self.config.robot_radius,
            enemy_robot_radius=self.config.enemy_robot_radius,
            forbidden_area_margin=self.config.forbidden_area_margin,
        )
        self.get_logger().info(
            f'ObstacleManager initialised: {len(manager._zones)} zones, {len(manager._elements)} elements'
        )
        return manager

    # ==================================== Callbacks ==========================================

    def _obstacle_states_callback(self, msg: StringMsg) -> None:
        states = json.loads(msg.data)
        changed = self.planner_avoidance.update_obstacle_states(states)
        changed = self.planner_straight.update_obstacle_states(states) or changed
        if changed:
            self.get_logger().info('[NAV] Obstacle states updated, planner cache invalidated')

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_pose = Pose2D(pose=msg.pose.pose)

    def enemy_odom_callback(self, msg: Odometry) -> None:
        pose = Pose2D(pose=msg.pose.pose)
        self.planner_avoidance.set_enemy_pose(pose.x, pose.y)
        self.planner_straight.set_enemy_pose(pose.x, pose.y)
        self.get_logger().info(f'Enemy pose received: ({pose.x:.2f}, {pose.y:.2f})', throttle_duration_sec=2.0)

    def collision_detector_callback(self, msg: Bool) -> None:
        self.collision_detected = msg.data

    # ==================================== Action Server ==========================================

    def navigate_callback(self, navigate_goal: Navigate.Goal) -> GoalResponse:
        yaw_deg = math.degrees(get_yaw(navigate_goal.pose))
        self.get_logger().info(
            f'[NAV] New Navigate request to pose: ({navigate_goal.pose.position.x:.3f}, '
            f'{navigate_goal.pose.position.y:.3f}, {yaw_deg:.1f}deg), '
            f'timeout={navigate_goal.timeout:.1f}s'
        )

        with self.state_lock:
            self.current_navigate_goal = navigate_goal
            if self.planning:
                self.get_logger().warn('[NAV] Preempting previous goal for new one!')
                self.active_planner.cancel()
                self.new_goal_waiting = True

            self.planning = True
            if navigate_goal.use_collision_avoidance:
                self.active_planner = self.planner_avoidance
                self.get_logger().info('[NAV] Collision avoidance ENABLED for this goal')
            else:
                self.active_planner = self.planner_straight
                self.get_logger().info('[NAV] Collision avoidance DISABLED for this goal')
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        with self.state_lock:
            if self.planning:
                self.active_planner.cancel()
                self.new_goal_waiting = True
                self.get_logger().info('[NAV] Cancel accepted, signalling execute_callback to stop')
            else:
                self.get_logger().warn('[NAV] No active goal to cancel!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle) -> Navigate.Result:
        with self.mutex_exec:
            with self.state_lock:
                self.new_goal_waiting = False
            
            self.active_planner.start(self.current_navigate_goal)
            action_state = ActionState.RUNNING

            while rclpy.ok() and goal_handle.is_active and action_state == ActionState.RUNNING and not self.collision_detected:
                with self.state_lock:
                    if self.new_goal_waiting:
                        break

                t_loop_start = time.time()
                self.exec_time_measurer.start()

                output = self.active_planner.step(self.robot_pose)
                
                # Retrieve pure state directive without passing goal_handle
                process_result = self._process_planner_output(output)
                action_state = process_result.state
                
                if process_result.feedback:
                    goal_handle.publish_feedback(process_result.feedback)

                self.exec_time_measurer.stop()
                
                sleep_time = max(0.0, self.config.loop_period - (time.time() - t_loop_start))
                time.sleep(sleep_time)

            # Finalize and cleanup
            result = self._finalize_action(goal_handle, action_state, process_result.message)
            
            with self.state_lock:
                if not self.new_goal_waiting:
                    self.planning = False

            return result

    # ==================================== Execution Logic ==========================================

    def _process_planner_output(self, output: any) -> ProcessResult:
        """Evaluates planner state machine output and returns decoupled directives."""
        if output.status == PlannerStatus.INITIALIZING:
            self.get_logger().info('[NAV] Waiting for robot pose', throttle_duration_sec=1.0)
            return ProcessResult(
                state=ActionState.RUNNING, 
                feedback=get_feedback_msg(_INITIALIZING_FEEDBACK_RESULT, [], 0)
            )

        if output.status == PlannerStatus.TIMED_OUT:
            self.get_logger().warn('[NAV] TIMEOUT reached! Aborting goal.')
            if output.send_stop:
                self.pose_controller_manager.publish_stop()
            return ProcessResult(state=ActionState.ABORTED, message='Timeout!')

        if output.status == PlannerStatus.IN_FORBIDDEN_AREA:
            self.get_logger().info(f'[NAV] Robot is in forbidden area at ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})', throttle_duration_sec=1.0)
            if output.send_stop:
                self.pose_controller_manager.publish_stop()
            elif output.ctrl_goal is not None:
                self.pose_controller_manager.publish_ctrl_goal(
                    output.ctrl_goal, metadata=self.current_navigate_goal, is_waypoint=output.is_waypoint
                )
            return ProcessResult(state=ActionState.RUNNING)

        if output.status == PlannerStatus.NO_PATH:
            self.get_logger().info(f'[NAV] No path found from robot=({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}) to goal=({self.current_navigate_goal.pose.position.x:.2f}, {self.current_navigate_goal.pose.position.y:.2f})!', throttle_duration_sec=1.0)
            return ProcessResult(
                state=ActionState.RUNNING, 
                feedback=get_feedback_msg(output.path_result, [], output.max_linear_speed)
            )

        if output.status == PlannerStatus.RUNNING:
            if output.send_stop:
                self.get_logger().info('[NAV] Enemy detected: waiting before backward step', throttle_duration_sec=1.0)
                self.pose_controller_manager.publish_stop()
            elif output.ctrl_goal is not None:
                self.get_logger().info(f'[NAV] Executing path... robot=({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f}) -> next WP=({output.ctrl_goal.x:.2f}, {output.ctrl_goal.y:.2f})', throttle_duration_sec=1.0)
                self.pose_controller_manager.publish_ctrl_goal(
                    output.ctrl_goal, metadata=self.current_navigate_goal, is_waypoint=output.is_waypoint
                )
            self.publish_path([p.to_ros_pose() for p in output.remaining_path])
            self.active_planner.draw_viz(output.waypoints, output.waypoint_idx)
            
            return ProcessResult(
                state=ActionState.RUNNING, 
                feedback=get_feedback_msg(output.path_result, output.remaining_path, output.max_linear_speed)
            )

        if output.status == PlannerStatus.GOAL_REACHED:
            self.get_logger().info(f'[NAV] FINAL GOAL REACHED at robot=({self.robot_pose.x:.3f}, {self.robot_pose.y:.3f})')
            return ProcessResult(state=ActionState.SUCCEEDED, message='Goal reached!')

        return ProcessResult(state=ActionState.ABORTED, message='Unknown planner status')

    def _finalize_action(self, goal_handle: ServerGoalHandle, final_state: ActionState, msg: str) -> Navigate.Result:
        """Cleans up controllers and sets final Action Server terminal state based on directives."""
        self.pose_controller_manager.publish_stop()
        self.publish_path([])

        if final_state == ActionState.SUCCEEDED:
            result = Navigate.Result(success=True, message=msg)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.succeed()
            self.get_logger().info('[NAV] RESULT => Goal REACHED')
            return result

        if self.collision_detected:
            result = Navigate.Result(success=False, message='Aborted due to collision detected!')

        with self.state_lock:
            was_preempted = self.new_goal_waiting

        if was_preempted:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().info('[NAV] RESULT => Goal preempted (new goal or cancel)')
        elif final_state == ActionState.ABORTED:
            if goal_handle.is_active:
                goal_handle.abort()
            self.get_logger().error(f'[NAV] RESULT => Aborted ({msg})')
        elif not goal_handle.is_active:
            self.get_logger().info('[NAV] RESULT => Goal finished (internal)')
        else:
            goal_handle.abort()
            self.get_logger().error(f'[NAV] RESULT => Unknown exit state: {final_state}')

        return Navigate.Result(success=False, message=msg or 'Goal aborted!')

    # ====================================== Visualization & Diagnostics ==========================================

    def viz_timer_callback(self) -> None:
        with self.state_lock:
            is_planning = self.planning
            
        if not is_planning:
            self.active_planner.draw_viz(None, 0)

    def _produce_planning_diagnostics(self, stat: diagnostic_updater.DiagnosticStatusWrapper) -> diagnostic_updater.DiagnosticStatusWrapper:
        metrics = self.active_planner.planning_metrics
        if metrics.last_ms is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "No planning call yet")
            return stat

        level = diagnostic_msgs.msg.DiagnosticStatus.OK
        if metrics.worst_ms > 75.0 or metrics.fail_rate > 0.2:
            level = diagnostic_msgs.msg.DiagnosticStatus.WARN

        stat.summary(level, f"last={metrics.last_ms:.1f}ms  avg={metrics.avg_ms:.1f}ms  worst={metrics.worst_ms:.1f}ms")
        stat.add("Last planning time (ms)", f"{metrics.last_ms:.2f}")
        stat.add("Avg planning time (ms)", f"{metrics.avg_ms:.2f}")
        stat.add("Worst planning time (ms)", f"{metrics.worst_ms:.2f}")
        stat.add("Total calls", str(metrics.n_calls))
        stat.add("Failed calls (no path)", str(metrics.n_failed))
        stat.add("Failure rate", f"{metrics.fail_rate * 100:.1f}%")
        return stat

    def publish_path(self, path: List[Pose]) -> None:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        path_msg.poses = [self._pose_to_pose_stamped(pose, 'odom') for pose in path]
        self.path_publisher_viz.publish(path_msg)

    def _pose_to_pose_stamped(self, pose: Pose, frame_id: str) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = pose
        return pose_stamped




def main(args: Optional[List[str]] = None) -> None:
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
