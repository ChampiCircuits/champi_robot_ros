#!/usr/bin/env python3
"""
State Machine ROS Node - Thin ROS wrapper
Connects the pure state machine to ROS interfaces.
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from typing import Optional

from champi_interfaces.msg import STMState
from std_msgs.msg import Int8, Int8MultiArray, String, Empty, Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import time
from math import atan2, pi, sqrt

from champi_brain.strategy_dsl import Action, MotionParams, Position, Offset
from champi_brain.enums import Color
from champi_brain.state_machine import StateMachine, StateMachineConfig
from champi_brain.core.match_controller import MatchController
from champi_brain.ros.ros_action_executor import ROSActionExecutor
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
        strategy_file = self.declare_parameter('strategy_file', '').value
        use_default_strategy = self.declare_parameter('use_above_default_strategy', False).value
        self.sim_mode = self.declare_parameter('sim', False).value
        
        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  strategy_file: {strategy_file}')
        self.get_logger().info(f'  use_default_strategy: {use_default_strategy}')
        self.get_logger().info(f'  sim_mode: {self.sim_mode}')
        
        # ============================================================
        # CREATE CORE COMPONENTS
        # ============================================================
        
        # Match controller (100s match)
        self.match_controller = MatchController(total_time=100.0) # TODO make param
        self.match_controller.on_score_changed = self._on_score_changed
        
        # ROS Action Executor
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
        # TIMER
        # ============================================================
        
        # Main update loop (10 Hz)
        self.create_timer(0.1, self._update_callback)
        
        # ============================================================
        # AUTO-START IN SIM MODE
        # ============================================================
        
        # Store for later use
        self.use_default_strategy = use_default_strategy
        self.strategy_file = strategy_file
        
        if use_default_strategy and self.sim_mode and strategy_file:
            self.get_logger().warn(f'🎮 SIM MODE: Auto-loading strategy: {strategy_file}')
            # Default to YELLOW team in simulation
            self._load_strategy(strategy_file, 'YELLOW') # TODO make color a param
        
        self.get_logger().warn('✅ State Machine ready!')
    
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
        
        self.get_logger().info(f'🎯 Score: {new_score} points')
    
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
                self.get_logger().warn('🤖 Simulation mode: auto-releasing tirette')
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
        """Set initial robot pose in localization."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta_rad = pose[2] * pi / 180.0
        from math import sin, cos
        msg.pose.pose.orientation.z = sin(theta_rad / 2.0)
        msg.pose.pose.orientation.w = cos(theta_rad / 2.0)
        
        # TODO: Publish to /initialpose or call SetPose service
        self.get_logger().info(f'📍 Initial pose set: {pose}')
    
    def _on_state_changed(self, new_state: str) -> None:
        """Called when state machine changes state."""
        self.get_logger().info(f'🔄 State: {new_state}')
    
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
