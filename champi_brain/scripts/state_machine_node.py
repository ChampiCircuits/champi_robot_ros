#!/usr/bin/env python3
"""
State Machine ROS Node - Thin ROS wrapper
Connects the pure state machine to ROS interfaces.
"""
# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
# Messages imports
from champi_interfaces.msg import STMState
from std_msgs.msg import Int8, Int8MultiArray, String, Empty, Float32, Bool
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from champi_interfaces.srv import SetPose
# Other imports
import time
from math import atan2, pi
from typing import Optional
# champi_brain imports
from champi_brain.strategy_dsl import Action, MotionParams, Position, Offset
from champi_brain.enums import Color
from champi_brain.state_machine import StateMachine, StateMachineConfig
from champi_brain.match_controller import MatchController
from champi_brain.action_executor.action_executor import ActionExecutor
from champi_brain.action_executor.ros_action_executor import ROSActionExecutor
from champi_brain.action_executor.sim_action_executor import SIMActionExecutor
from champi_brain.strategy_loader import load_strategy


class StateMachineNode(Node):
    """
    ROS2 node that wraps the pure state machine.
    Responsibilities:
    - Create and configure StateMachine with ROS dependencies
    - Convert ROS messages to state machine calls
    - Publish state machine status to ROS topics
    """
    
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info('🚀 Launching State Machine...')
        
        # ============================================================
        # PARAMETERS
        # ============================================================     
        self.declare_parameter('default_strategy_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('use_default_strategy_and_color_in_sim', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('sim', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('default_sim_color', rclpy.Parameter.Type.STRING)
        self.declare_parameter('match_total_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('return_home_safety_margin', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('simulate_actuators_delays', rclpy.Parameter.Type.BOOL)

        self.default_strategy_file = self.get_parameter('default_strategy_file').value
        self.use_default_strategy_and_color_in_sim = self.get_parameter('use_default_strategy_and_color_in_sim').value
        self.sim_mode = self.get_parameter('sim').value
        self.default_sim_color = self.get_parameter('default_sim_color').value
        match_total_time = self.get_parameter('match_total_time').value
        return_home_safety_margin = self.get_parameter('return_home_safety_margin').value
        simulate_actuators_delays = self.get_parameter('simulate_actuators_delays').value

        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  sim_mode: {self.sim_mode}')
        self.get_logger().info(f'  use_default_strategy_and_color_in_sim: {self.use_default_strategy_and_color_in_sim}')
        if self.use_default_strategy_and_color_in_sim:
            self.get_logger().info(f'  default_sim_color: {self.default_sim_color}')
            self.get_logger().info(f'  default_strategy_file: {self.default_strategy_file}')
            self.get_logger().info(f'  simulate_actuators_delays: {simulate_actuators_delays}')
        self.get_logger().info(f'  match_total_time: {match_total_time}s')
        self.get_logger().info(f'  return_home_safety_margin: {return_home_safety_margin}s')
        
        # ============================================================
        # CONFIGURE MOTION DEFAULTS FROM POSE CONTROLLER PARAMS
        # ============================================================
        self._configure_motion_defaults()
        
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
        self.action_executor.on_goal_failed = self._on_action_failed
        
        # State Machine Config (will be filled when strategy is chosen)
        self.sm_config = None
        self.state_machine: Optional[StateMachine] = None
        
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
        self.current_pose = None  # (x, y, theta_deg)
        
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
        
        # Platform detection # TODO remove
        self.create_subscription(
            Float32, '/platform_distance',
            self._on_platform_distance,
            10
        )
        self.platform_distance = None
        
        # Actuators finished
        self.create_subscription(
            Int8MultiArray, '/actuators_finished',
            self._on_actuators_finished,
            10
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
        
        # ============================================================
        # TIMER
        # ============================================================
        
        # Main update loop (10 Hz)
        self.create_timer(0.1, self._update_callback)
        
        # ============================================================
        # AUTO-START IN SIM MODE
        # ============================================================
                
        if self.sim_mode and self.use_default_strategy_and_color_in_sim:
            self.get_logger().warn(f'🎮 SIM MODE: Auto-loading strategy: {self.default_strategy_file}')
            self._load_strategy(self.default_strategy_file, self.default_sim_color)
        
        self.get_logger().warn('✅ State Machine ready!\n')

    
    def _configure_motion_defaults(self) -> None:
        """Configure default motion parameters from ROS parameters."""
        params = {
            'default_motion_speed': rclpy.Parameter.Type.DOUBLE,
            'default_motion_end_speed': rclpy.Parameter.Type.DOUBLE,
            'default_motion_accel_linear': rclpy.Parameter.Type.DOUBLE,
            'default_motion_accel_angular': rclpy.Parameter.Type.DOUBLE,
            'default_motion_use_dynamic_layer': rclpy.Parameter.Type.BOOL
        }
        
        values = {}
        for name, param_type in params.items():
            self.declare_parameter(name, param_type)
            param = self.get_parameter(name)
            if param.type_ == rclpy.Parameter.Type.NOT_SET:
                raise RuntimeError(f"Required parameter '{name}' not found in config under 'state_machine'")
            values[name] = param.value
        
        MotionParams.set_defaults(
            speed=values['default_motion_speed'],
            end_speed=values['default_motion_end_speed'],
            accel_linear=values['default_motion_accel_linear'],
            accel_angular=values['default_motion_accel_angular'],
            use_dynamic_layer=values['default_motion_use_dynamic_layer']
        )
        
        self.get_logger().info(
            f"Motion defaults: speed={values['default_motion_speed']}, "
            f"end_speed={values['default_motion_end_speed']}, "
            f"accel_linear={values['default_motion_accel_linear']}, "
            f"accel_angular={values['default_motion_accel_angular']}, "
            f"use_dynamic_layer={values['default_motion_use_dynamic_layer']}"
        )

    # ================================================================
    # ROS CALLBACKS
    # ================================================================

    def _on_stm_state(self, msg: STMState) -> None:
        """Handle STM state (e-stop and tirette)."""
        self.e_stop_pressed = msg.e_stop_pressed
        self.tirette_released = msg.tirette_released
        
        if not self.state_machine:
            return
        
        # Handle e-stop
        if self.e_stop_pressed:
            self.get_logger().error('🛑 E-STOP PRESSED!')
            self.state_machine.request_stop()
        
        # Handle tirette in init state
        if self.tirette_released and self.state_machine.get_state() == StateMachine.STATE_INIT:
            self.get_logger().warn('🏁 TIRETTE RELEASED - Starting match!')
            self.state_machine.notify_tirette_released()
    
    def _on_odometry(self, msg: Odometry) -> None:
        """Handle odometry updates."""
        # Convert quaternion to euler
        theta_rad = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta_deg = theta_rad * 180.0 / pi
        
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta_deg
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
        
        if self.state_machine:
            self.state_machine.reset()
        
        if self.action_executor:
            self.action_executor.cancel_current_action()
        
        if self.match_controller:
            self.match_controller.reset()
        
        # Reset actuators
        if not self.sim_mode:
            time.sleep(0.5)
            self.action_executor.execute_actuator_action('ENABLE_ALL_MOTORS')
            time.sleep(1.0)
            self.action_executor.execute_actuator_action('RESET_ACTUATORS')
        
        self.get_logger().warn('✅ State machine reset complete')
    
    def _on_platform_distance(self, msg: Float32) -> None:
        """Handle platform distance detection."""
        self.platform_distance = msg.data
        
        if not self.state_machine or not self.current_pose:
            return
        
        # Only process if we're in detection mode
        if self.state_machine.get_state() != StateMachine.STATE_EXECUTING_ACTION:
            return
        
        # Check if platform detected
        if self.platform_distance is not None and self.platform_distance > 0 and self.platform_distance < 0.6:
            # Compute platform pose from robot pose
            x_robot, y_robot, theta_deg = self.current_pose
            theta_rad = theta_deg * pi / 180.0
            
            # Platform is at distance in front of robot
            half_platform = 0.05
            center_dist = self.platform_distance + half_platform
            
            from math import cos, sin
            x_platform = x_robot + center_dist * cos(theta_rad)
            y_platform = y_robot + center_dist * sin(theta_rad)
            
            self.get_logger().info(f'📍 Platform detected at ({x_platform:.2f}, {y_platform:.2f})')
            self.state_machine.notify_platform_detected((x_platform, y_platform, theta_deg))
    
    def _on_actuators_finished(self, msg: Int8MultiArray) -> None:
        """Handle actuator completion."""
        if not self.state_machine:
            return
        
        self.get_logger().debug(f'🤖 Actuators finished: {msg.data}')
        self.state_machine.notify_action_completed()
    
    # ================================================================
    # STATE MACHINE CALLBACKS
    # ================================================================
    
    def _on_action_completed(self) -> None:
        """Called when action executor completes an action."""
        if self.state_machine:
            self.state_machine.notify_action_completed()
    
    def _on_action_failed(self, error_msg: str) -> None:
        """Called when action executor fails."""
        self.get_logger().error(f'❌ Action failed: {error_msg}')
        
        # For now, treat as completion (could implement retry logic)
        if self.state_machine:
            self.state_machine.notify_action_completed()
    
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
        if not self.state_machine:
            # Publish "waiting" state when no strategy loaded
            msg = String()
            msg.data = "waiting_for_strategy"
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
        msg.data = self.state_machine.get_state()
        self.state_pub.publish(msg)
    
    # ================================================================
    # HELPER METHODS
    # ================================================================
    
    def _load_strategy(self, strategy_file: str, color: str) -> None:
        """Load strategy and create state machine."""
        try:
            # Load strategy from file
            strategy_path = get_package_share_directory('champi_brain') + '/strategies/' + strategy_file
            strategy, init_pose, home_pose, wait_home_pose = load_strategy(
                strategy_path,
                color,
                self.get_logger()
            )
            
            self.get_logger().info(f'✅ Strategy loaded:')
            self.get_logger().info(f'   Init pose: {init_pose}')
            self.get_logger().info(f'   Home pose: {home_pose}')
            self.get_logger().info(f'   Wait home pose: {wait_home_pose}')
            self.get_logger().info(f'   Actions: {len(strategy)}')
            
            # Create config
            self.sm_config = StateMachineConfig(
                color=color,
                init_pose=tuple(init_pose),
                home_pose=tuple(home_pose),
                wait_to_come_home_pose=tuple(wait_home_pose),
                simulation_mode=self.sim_mode
            )
            
            # Create state machine
            self.state_machine = StateMachine(
                executor=self.action_executor,
                match_controller=self.match_controller,
                config=self.sm_config
            )
            
            # Set strategy
            self.state_machine.set_strategy(strategy)
            
            # Set callbacks
            self.state_machine.on_state_changed = self._on_state_changed
            self.state_machine.on_strategy_completed = self._on_strategy_completed
            
            # Initialize robot pose
            self._set_initial_pose(init_pose)
            
            # Start initialization
            self.state_machine.start_initialization()
            self.state_machine.notify_ros_initialized()
            self.state_machine.notify_config_chosen()
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load strategy: {e}')
            import traceback
            traceback.print_exc()
    
    def _set_initial_pose(self, pose: list) -> None:
        """Set initial robot pose in localization via SetPose service."""
        self.get_logger().warn(f'📍 Setting initial pose via /set_pose service: ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.1f}°)')
        # Wait for service to be available
        if not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('⚠️ /set_pose service not available, pose not set')
            exit(1) # TODO better error handling
        
        # Create request
        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = pose[0]
        request.pose.pose.pose.position.y = pose[1]
        request.pose.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta_rad = pose[2] * pi / 180.0
        from math import sin, cos
        request.pose.pose.pose.orientation.z = sin(theta_rad / 2.0)
        request.pose.pose.pose.orientation.w = cos(theta_rad / 2.0)
        
        # Set covariance (small uncertainty for initial pose) # TODO make param ?
        request.pose.pose.covariance[0] = 1e-5   # x variance
        request.pose.pose.covariance[7] = 1e-5   # y variance
        request.pose.pose.covariance[35] = 1e-5  # yaw variance
        
        # Call service asynchronously
        future = self.set_pose_client.call_async(request)
        
    
    def _on_state_changed(self, new_state: str) -> None:
        """Called when state machine changes state."""
        # self.get_logger().debug(f'🔄 New state: {new_state}')
        pass
    
    def _on_strategy_completed(self) -> None:
        """Called when all strategy actions are complete."""
        self.get_logger().warn('✅ Strategy completed!')


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
