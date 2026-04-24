#!/usr/bin/env python3
"""
State Machine ROS Node - Thin ROS wrapper
Connects the pure state machine to ROS interfaces.
"""
# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
# Messages imports
from std_msgs.msg import Int8, Int8MultiArray, String, Empty, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration

from champi_interfaces.msg import STMState, TableObservation
from champi_interfaces.srv import SetPose
from champi_interfaces.srv import SetAutoPlacementEnabled
# Other imports
import time
from math import atan2, degrees, radians, cos, sin
# champi_brain imports
from champi_brain.state_machine import StateMachine, StrategyConfig
from champi_brain.match_controller import MatchController
from champi_brain.action_executor.action_executor import ActionExecutor
from champi_brain.action_executor.ros_action_executor import ROSActionExecutor
from champi_brain.action_executor.sim_action_executor import SIMActionExecutor
from champi_brain.strategy_loader import load_strategy
from champi_brain.motion_config import configure_motion_defaults
from champi_brain.actuator_commands import ActuatorCommand
from champi_brain.strategy_dsl import MotionParams
from champi_brain.auto_placement_controller import AutoPlacementController
from champi_libraries_py.marker_helper.canva import *
from champi_libraries_py.marker_helper.canva import Canva



class StateMachineNode(Node):
    """
    ROS2 node that wraps the pure state machine.
    Responsibilities:
    - Create and configure StateMachine with ROS dependencies
    - Convert ROS messages to state machine calls
    - Publish state machine status to ROS topics
    """
    
    def __init__(self):
        super().__init__('state_machine_node', namespace='champi_brain')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info('🚀 Launching State Machine...')

        Canva(self, enable=True)

        # ============================================================
        # PARAMETERS
        # ============================================================     
        # Declare node-specific parameters (from champi_brain/state_machine_node config section)
        self.declare_parameter('default_strategy_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('use_default_strategy_and_color_in_sim', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('default_sim_color', rclpy.Parameter.Type.STRING)
        self.declare_parameter('match_total_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('return_home_safety_margin', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('simulate_actuators_delays', rclpy.Parameter.Type.BOOL)
        # Declare shared parameter (from champi_brain namespace)
        self.declare_parameter('initial_world_state_file', rclpy.Parameter.Type.STRING)
        # Declare general parameters (from /** or top-level config)
        self.declare_parameter('sim', rclpy.Parameter.Type.BOOL)

        self.default_strategy_file = self.get_parameter('default_strategy_file').value
        self.use_default_strategy_and_color_in_sim = self.get_parameter('use_default_strategy_and_color_in_sim').value
        self.sim_mode = self.get_parameter('sim').value
        self.default_sim_color = self.get_parameter('default_sim_color').value
        match_total_time = self.get_parameter('match_total_time').value
        return_home_safety_margin = self.get_parameter('return_home_safety_margin').value
        simulate_actuators_delays = self.get_parameter('simulate_actuators_delays').value
        self.initial_world_state_file = self.get_parameter('initial_world_state_file').value

        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'\tsim_mode: {self.sim_mode}')
        self.get_logger().info(f'\tuse_default_strategy_and_color_in_sim: {self.use_default_strategy_and_color_in_sim}')
        if self.use_default_strategy_and_color_in_sim:
            self.get_logger().info(f'\tdefault_sim_color: {self.default_sim_color}')
            self.get_logger().info(f'\tdefault_strategy_file: {self.default_strategy_file}')
            self.get_logger().info(f'\tsimulate_actuators_delays: {simulate_actuators_delays}')
        self.get_logger().info(f'\tmatch_total_time: {match_total_time}s')
        self.get_logger().info(f'\treturn_home_safety_margin: {return_home_safety_margin}s')

        # Auto-placement constants (hardcoded on purpose for now)
        self.AUTO_PLACEMENT_TOTAL_TIMEOUT_S = 15.0
        self.AUTO_PLACEMENT_REST_REQUIRED_S = 0.5
        self.AUTO_PLACEMENT_REQUIRED_ARUCO_POSES = 3
        self.AUTO_PLACEMENT_ARUCO_POS_THRESHOLD_M = 0.05
        self.AUTO_PLACEMENT_ARUCO_ANGLE_THRESHOLD_DEG = 8.0
        self.AUTO_PLACEMENT_LINEAR_VEL_THRESHOLD = 0.03
        self.AUTO_PLACEMENT_ANGULAR_VEL_THRESHOLD = 0.15
        
        # ============================================================
        # CONFIGURE MOTION DEFAULTS FROM POSE CONTROLLER PARAMS
        # ============================================================
        configure_motion_defaults(self)
        
        # ============================================================
        # CREATE CORE COMPONENTS
        # ============================================================
        
        # Match controller with timing parameters
        self.match_controller = MatchController(total_time=match_total_time, return_home_safety_margin=return_home_safety_margin)
        self.match_controller.on_score_changed = self._on_score_changed
        
        # Action Executor
        self.action_executor: ActionExecutor
        if self.sim_mode:
            self.action_executor = SIMActionExecutor(self, simulate_actuators_delays)
        else:
            self.action_executor = ROSActionExecutor(self)
        self.action_executor.on_goal_reached = self._on_action_completed
        self.action_executor.on_goal_failed = self._on_action_failed # TODO utiliser les 4 callbacks 
        
        # State Machine Config (will be filled when strategy is chosen)
        self.sm_strategy_config = None
        # Create state machine
        self.state_machine = StateMachine( 
            executor=self.action_executor,
            match_controller=self.match_controller
        )
        
        # ============================================================
        # ROS SUBSCRIBERS
        # ============================================================
        
        # STM state (e-stop + tirette)
        self.create_subscription(
            STMState, '/STM_state',
            self._on_stm_state,
            10
        )
        self.e_stop_pressed = False
        self.tirette_released = False
        
        # Odometry
        self.create_subscription(
            Odometry, '/odom',
            self._on_odometry,
            10
        )
        self.current_pose = None  # (x, y, theta_deg) # TODO object Position instead?
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Chosen strategy
        self.create_subscription(
            String, '/chosen_strategy',
            self._on_chosen_strategy,
            10
        )
        
        # Reset command
        self.create_subscription(
            Empty, '/reset_state_machine',
            self._on_reset_request,
            10
        )
        
        # Box detection
        self.create_subscription(
            PoseStamped, '/nutboxes_relative_position',
            self._on_nutbox_pose,
            10
        )
        self.last_nutbox_pose: PoseStamped | None = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/aruco_loc/pose',
            self._on_aruco_pose,
            10
        )
        
        # Actuators finished
        self.create_subscription(
            Int8MultiArray, '/actuators_finished',
            self._on_actuators_finished,
            10
        )

        # World state - Subscribe with TRANSIENT_LOCAL to receive last message
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Receive last published message
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(
            TableObservation,
            '/world_state',
            self._on_world_state,
            latched_qos
        )

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================
        
        # Score
        self.score_pub = self.create_publisher(Int8, '/final_score', 10)
        
        # State (for debugging/monitoring)
        self.state_pub = self.create_publisher(String, '/sm_state', 10)
        
        # ============================================================
        # ROS SERVICE CLIENTS
        # ============================================================
        
        # Service client for setting initial pose
        self.set_pose_client = self.create_client(SetPose, '/set_pose')

        # Service server for toggling auto placement from UI
        self.create_service(
            SetAutoPlacementEnabled,
            '/set_auto_placement_enabled',
            self._on_set_auto_placement_enabled
        )

        self.auto_placement = AutoPlacementController(
            logger=self.get_logger(),
            request_set_pose=self._set_initial_pose,
            request_move_to_init=self._request_auto_placement_move,
            on_completed=self._on_auto_placement_completed,
            total_timeout_s=self.AUTO_PLACEMENT_TOTAL_TIMEOUT_S,
            rest_required_s=self.AUTO_PLACEMENT_REST_REQUIRED_S,
            required_aruco_poses=self.AUTO_PLACEMENT_REQUIRED_ARUCO_POSES,
            aruco_pos_threshold_m=self.AUTO_PLACEMENT_ARUCO_POS_THRESHOLD_M,
            aruco_angle_threshold_deg=self.AUTO_PLACEMENT_ARUCO_ANGLE_THRESHOLD_DEG,
            linear_vel_threshold=self.AUTO_PLACEMENT_LINEAR_VEL_THRESHOLD,
            angular_vel_threshold=self.AUTO_PLACEMENT_ANGULAR_VEL_THRESHOLD,
        )
        
        # ============================================================
        # TIMER
        # ============================================================
        
        # Main update loop (10 Hz)
        self.create_timer(0.1, self._update_callback)
        
        # ============================================================
        # AUTO-START IN SIM MODE (deferred to first timer tick so the
        # node is already being spun by the executor, which allows
        # service calls with spin_once to work correctly)
        # ============================================================
        self._pending_auto_load = False
        if self.sim_mode and self.use_default_strategy_and_color_in_sim:
            self.get_logger().warn(f'🎮 SIM MODE: Will auto-load strategy: {self.default_strategy_file}')
            self._pending_auto_load = True

        self.get_logger().warn('State Machine ready started!\n')


    # ================================================================
    # ROS CALLBACKS
    # ================================================================

    def _on_stm_state(self, msg: STMState) -> None:
        """Handle STM state (e-stop and tirette)."""
        self.e_stop_pressed = msg.e_stop_pressed
        self.tirette_released = msg.tirette_released
        
        if not self.state_machine: # TODO useful ? et les autres aussi
            return
        
        # Handle e-stop
        if self.e_stop_pressed:
            self.get_logger().error('🛑 E-STOP PRESSED!', throttle_duration_sec=2.)
            self.state_machine.request_stop()
        
        # Handle tirette in init state
        if self.tirette_released and self.state_machine.get_state() == StateMachine.STATE_INIT:
            self.get_logger().warn('🏁 TIRETTE RELEASED - Starting match!')
            self.state_machine.notify_tirette_released()
    
    def _on_odometry(self, msg: Odometry) -> None:
        """Handle odometry updates."""
        # Convert quaternion to euler
        theta_rad = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta_deg = degrees(theta_rad)
        
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta_deg
        )
        self.current_linear_velocity = (msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2) ** 0.5
        self.current_angular_velocity = abs(msg.twist.twist.angular.z)

    def _on_set_auto_placement_enabled(self, request: SetAutoPlacementEnabled.Request, response: SetAutoPlacementEnabled.Response):
        """Enable/disable auto-placement from GUI."""
        self.auto_placement.set_enabled(bool(request.enabled))
        self.get_logger().warn(f'Auto-placement enabled set to: {self.auto_placement.enabled}')
        return response

    def _on_aruco_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Collect ArUco poses during localization phase."""
        theta_rad = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.auto_placement.on_aruco_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            degrees(theta_rad),
        )

    def _on_chosen_strategy(self, msg: String) -> None:
        """Handle strategy choice."""
        # Format: "strategy_file#COLOR"
        parts = msg.data.split('#')
        if len(parts) != 2:
            self.get_logger().error(f'Invalid strategy format: {msg.data}')
            return
        
        strategy_file, color = parts
        self.get_logger().info(f'📋 Chosen strategy: {strategy_file}, Color: {color}')
        
        self._load_strategy(strategy_file, color)
    
    def _on_reset_request(self, msg: Empty) -> None:
        """Handle reset request."""
        self.get_logger().warn('🔄 Reset requested')

        self.auto_placement.reset()
        
        if self.state_machine:
            self.state_machine.reset()
        
        if self.action_executor:
            self.action_executor.cancel_current_action()
        
        if self.match_controller:
            self.match_controller.reset()
        
        # Reset actuators
        if not self.sim_mode:
            time.sleep(0.5)
            self.action_executor.execute_actuator_action(ActuatorCommand.ENABLE_ALL_MOTORS)
            time.sleep(1.0)
            self.action_executor.execute_actuator_action(ActuatorCommand.RESET_ACTUATORS)
        
        self.get_logger().warn('✅ State machine reset complete')
    
    def _on_nutbox_pose(self, msg: PoseStamped) -> None:
        """Handle box relative pose detection (pose in base_link frame). z=-1 means no detection."""
        if not self.state_machine or not self.current_pose:
            return

        current_action = self.state_machine.current_action
        if current_action is None or current_action.action != ActuatorCommand.DETECT_NUTBOXES:
            return
        
        self.get_logger().info(f'📦 Nutbox pose received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}) in base_link frame')

        # Cancel timeout — we have a valid detection
        self.action_executor.cancel_detect_nutboxes_timeout()

        # Convert relative pose (base_link) to world pose using current robot pose
        x_robot, y_robot, theta_deg = self.current_pose
        theta_rad = radians(theta_deg)

        dx = msg.pose.position.x
        dy = msg.pose.position.y
        x_box = x_robot + cos(theta_rad) * dx - sin(theta_rad) * dy
        y_box = y_robot + sin(theta_rad) * dx + cos(theta_rad) * dy

        self.get_logger().info(f'📦 Nutboxes detected at ({x_box:.2f}, {y_box:.2f}, {theta_deg:.2f}°) in world frame')
        self.state_machine.notify_nutboxes_detected((x_box, y_box, theta_deg))
    
    def _on_actuators_finished(self, msg: Int8MultiArray) -> None:
        """Handle actuator completion."""
        if not self.state_machine:
            return
        
        self.get_logger().debug(f'🤖 Actuators finished: {msg.data}')
        self.state_machine.notify_action_completed()
    
    def _on_world_state(self, msg: TableObservation) -> None:
        """Handle world state updates."""
        self.get_logger().debug(f'🌍 World state received: {len(msg.detected_game_elements)} elements')
        
        # Convert TableObservation to dict of GameElement messages
        elements = {}
        for elem in msg.detected_game_elements:
            elements[elem.id] = elem
            self.get_logger().debug(f'\t {elem}')
        
        # Update state machine world state
        self.state_machine.update_world_state(elements)
    
    # ================================================================
    # STATE MACHINE CALLBACKS
    # ================================================================
    
    def _on_action_completed(self) -> None:
        """Called when action executor completes an action."""
        if self.auto_placement.in_progress and self.auto_placement.status == AutoPlacementController.STATUS_MOVING_TO_INIT:
            self.auto_placement.on_move_result(success=True)
            return

        if self.state_machine:
            self.state_machine.notify_action_completed()
    
    def _on_action_failed(self, error_msg: str) -> None:
        """Called when action executor fails."""
        self.get_logger().error(f'❌ Action failed: {error_msg}')

        if self.auto_placement.in_progress and self.auto_placement.status == AutoPlacementController.STATUS_MOVING_TO_INIT:
            self.auto_placement.on_move_result(success=False, error_msg=error_msg)
            return
        
        if self.state_machine:
            self.state_machine.cancel_current_group()
    
    def _on_score_changed(self, new_score: int) -> None:
        """Called when score changes."""
        msg = Int8()
        msg.data = new_score
        self.score_pub.publish(msg)
        self.get_logger().info(f'🎯 Score changed --> now: {new_score} points')
    
    # ================================================================
    # MAIN UPDATE LOOP
    # ================================================================
    
    def _update_callback(self) -> None:
        """Main update loop called at 10 Hz."""

        # Deferred auto-load: now the node is being spun by the executor,
        # so service calls (spin_once) will work properly.
        if self._pending_auto_load:
            self._pending_auto_load = False
            self.get_logger().warn(f'🎮 SIM MODE: Auto-loading strategy: {self.default_strategy_file}')
            self._load_strategy(self.default_strategy_file, self.default_sim_color)
            return

        if not self.state_machine:
            # Publish "waiting" state when no strategy loaded
            msg = String()
            msg.data = "waiting_for_strategy"
            self.state_pub.publish(msg)
            return

        if self.auto_placement.in_progress:
            self.auto_placement.update(self.current_linear_velocity, self.current_angular_velocity)
            msg = String()
            msg.data = self._sm_state_for_ui()
            self.state_pub.publish(msg)
            return
        
        # Auto-release tirette in sim mode
        if self.sim_mode and self.state_machine.get_state() == StateMachine.STATE_INIT:
            if not self.tirette_released:
                delay = 2.0  # seconds
                self.get_logger().warn(f'🎮 Simulation mode: auto-releasing tirette after {delay} seconds')
                self.get_clock().sleep_for(Duration(seconds=delay))
                self.get_logger().warn('🎮 Simulation mode: Tirette released !')
                self.tirette_released = True
                self.state_machine.notify_tirette_released()
        
        # Call state machine update
        self.state_machine.update()
        
        # Publish current state
        msg = String()
        msg.data = self._sm_state_for_ui()
        self.state_pub.publish(msg)
    
    # ================================================================
    # HELPER METHODS
    # ================================================================

    def _get_action_label(self) -> str:
        """Build a human-readable label for the current action being executed."""
        action = self.state_machine.last_dispatched_action
        if action is None:
            return ""

        label = action.action if isinstance(action.action, str) else action.action.name
        if action.group:
            label += f":[{action.group}]"

        return label

    def _sm_state_for_ui(self) -> str:
        if self.auto_placement.in_progress:
            return self.auto_placement.ui_state()
        return self.state_machine.get_state()

    def _request_auto_placement_move(self, init_pose: tuple[float, float, float]) -> None:
        x, y, theta_deg = init_pose
        motion = MotionParams(use_collision_avoidance=False)
        self.action_executor.move_to(x, y, theta_deg, motion)

    def _on_auto_placement_completed(self) -> None:
        if self.state_machine:
            self.state_machine.notify_config_chosen()

    def _load_strategy(self, strategy_file: str, color: str) -> None:
        """Load strategy and create state machine."""
        try:
            # Load strategy from file
            strategy_path = get_package_share_directory('champi_brain') + '/strategies/' + strategy_file
            world_state_path = get_package_share_directory('champi_brain') + '/config/' + self.initial_world_state_file
            
            strategy, init_pose, home_pose, wait_home_pose, time_per_action, points_per_action = load_strategy(
                strategy_path,
                color,
                self.get_logger(),
                world_state_path
            )
            
            self.get_logger().info(f'✅ Strategy loaded:')
            self.get_logger().info(f'   Init pose: {init_pose}')
            self.get_logger().info(f'   Home pose: {home_pose}')
            self.get_logger().info(f'   Wait home pose: {wait_home_pose}')
            self.get_logger().info(f'   Actions: {len(strategy)}')
            
            # Update time per action for simulation executor
            if self.sim_mode:
                self.action_executor.set_time_per_action(time_per_action)
            
            # Create config
            self.sm_strategy_config = StrategyConfig(
                color=color,
                init_pose=tuple(init_pose),
                home_pose=tuple(home_pose),
                wait_to_come_home_pose=tuple(wait_home_pose),
                come_home_points=points_per_action.get("COME_HOME", 10),
                simulation_mode=self.sim_mode
            )
            # Set strategy
            self.state_machine.set_strategy(strategy, self.sm_strategy_config)
            
            # Set callbacks
            self.state_machine.on_state_changed = self._on_state_changed
            self.state_machine.on_strategy_completed = self._on_strategy_completed

            # Start initialization
            self.state_machine.start_initialization()
            self.state_machine.notify_ros_initialized()

            if self.auto_placement.enabled and not self.sim_mode:
                self.auto_placement.start(tuple(init_pose))
            else:
                # Initialize robot pose directly when auto-placement is disabled.
                self._set_initial_pose(init_pose)
                self.state_machine.notify_config_chosen()
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load strategy: {e}')
            import traceback
            traceback.print_exc()
    
    def _set_initial_pose(self, pose: list) -> None:
        """Set initial robot pose in localization via SetPose service."""
        self.get_logger().warn(f'📍 Setting initial pose via /set_pose service: ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.1f}°)')
        
        # Check service availability (non-blocking: just check if it's known)
        if not self.set_pose_client.service_is_ready():
            self.get_logger().error('⚠️ /set_pose service not available yet')
            raise RuntimeError('/set_pose service not available')
        
        # Check that we have odometry (should already be available since
        # this is called from a spin callback, not from __init__)
        if self.current_pose is None:
            self.get_logger().warn('⚠️ No odometry received yet, but proceeding with set_pose anyway')
        
        # Create request
        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = pose[0]
        request.pose.pose.pose.position.y = pose[1]
        request.pose.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta_rad = radians(pose[2])
        from math import sin, cos
        request.pose.pose.pose.orientation.z = sin(theta_rad / 2.0)
        request.pose.pose.pose.orientation.w = cos(theta_rad / 2.0)
        
        # Set covariance (small uncertainty for initial pose) # TODO make param ?
        request.pose.pose.covariance[0] = 1e-5   # x variance
        request.pose.pose.covariance[7] = 1e-5   # y variance
        request.pose.pose.covariance[35] = 1e-5  # yaw variance
        
        # Call service asynchronously and check result via callback.
        # NOTE: We cannot use spin_once() here because this method may be
        # called from within a spin callback (e.g. _on_chosen_strategy or
        # _update_callback), and the single-threaded executor cannot re-enter.
        # Instead, we fire the request and attach a done-callback.
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(self._on_set_pose_done)
        self.get_logger().info('📤 /set_pose request sent, waiting for response asynchronously...')

    def _on_set_pose_done(self, future) -> None:
        """Callback when the /set_pose service call completes."""
        try:
            future.result()
            self.get_logger().info('✅ Initial pose set successfully')
            self.auto_placement.on_set_pose_done(success=True)
        except Exception as e:
            self.get_logger().error(f'❌ /set_pose service call failed: {e}')
            self.auto_placement.on_set_pose_done(success=False, error_msg=str(e))

    
    def _on_state_changed(self, new_state: str) -> None:
        """Called when state machine changes state."""
        # self.get_logger().debug(f'🔄 New state: {new_state}')

        # Display current action as text in RViz
        Canva().clear()
        Canva().add(items.Text((1.5, 1.5, 0.0), text=self._get_action_label(), size=0.15, color=presets.BLUE))
        Canva().draw()
        pass
    
    def _on_strategy_completed(self) -> None:
        """Called when all strategy actions are complete."""
        self.get_logger().warn('✅ Strategy completed!')
        Canva().clear()
        Canva().add(items.Text((1.5, 1.5, 0.0), text="Strategy completed!", size=0.15, color=presets.GREEN))
        Canva().draw()

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = StateMachineNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
