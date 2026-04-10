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
from nav_msgs.msg import Odometry
from rclpy.duration import Duration

from champi_interfaces.msg import STMState, TableObservation
from champi_interfaces.srv import SetPose
# Other imports
import time
from math import atan2, degrees, radians
# champi_brain imports
from champi_brain.state_machine import StateMachine, StrategyConfig
from champi_brain.match_controller import MatchController
from champi_brain.action_executor.action_executor import ActionExecutor
from champi_brain.action_executor.ros_action_executor import ROSActionExecutor
from champi_brain.action_executor.sim_action_executor import SIMActionExecutor
from champi_brain.strategy_loader import load_strategy
from champi_brain.motion_config import configure_motion_defaults
from champi_brain.actuator_commands import ActuatorCommand
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
        self.last_platform_distance = None
        
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
            self.action_executor.execute_actuator_action(ActuatorCommand.ENABLE_ALL_MOTORS)
            time.sleep(1.0)
            self.action_executor.execute_actuator_action(ActuatorCommand.RESET_ACTUATORS)
        
        self.get_logger().warn('✅ State machine reset complete')
    
    def _on_platform_distance(self, msg: Float32) -> None:
        """Handle platform distance detection.""" # TODO informer le world state ?
        self.last_platform_distance = msg.data
        
        if not self.state_machine or not self.current_pose:
            return
        
        # Only process if we're in detection mode
        if self.state_machine.get_state() != StateMachine.STATE_EXECUTING_ACTION:
            return
        
        # Check if platform detected
        if self.last_platform_distance is not None and self.last_platform_distance > 0 and self.last_platform_distance < 0.6:
            # Compute platform pose from robot pose
            x_robot, y_robot, theta_deg = self.current_pose
            theta_rad = radians(theta_deg)
            
            # Platform is at distance in front of robot
            half_platform = 0.05
            center_dist = self.last_platform_distance + half_platform
            
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
        if self.state_machine:
            self.state_machine.notify_action_completed()
    
    def _on_action_failed(self, error_msg: str) -> None:
        """Called when action executor fails."""
        self.get_logger().error(f'❌ Action failed: {error_msg}')
        
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

    def _get_action_label(self) -> str:
        """Build a human-readable label for the current action being executed."""
        action = self.state_machine.last_dispatched_action
        if action is None:
            return ""

        label = action.action.name  # e.g. "MOVE", "TAKE_2_BOXES"...
        if action.group:
            label += f":[{action.group}]"

        return label

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
        if not self.set_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('⚠️ /set_pose service not available after 5s timeout')
            raise RuntimeError('/set_pose service not available')
        
        # Wait for first odometry message to ensure localization system is ready
        # This prevents the pose from being overwritten by initialization values from loc_node
        if self.current_pose is None:
            self.get_logger().info('⏱️ Waiting for first odometry message...')
            timeout = 5.0  # seconds
            start_time = time.time()
            while self.current_pose is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.current_pose is None:
                self.get_logger().error('⚠️ No odometry received after 5s timeout')
                raise RuntimeError('No odometry received, cannot set initial pose')
            
            self.get_logger().info(f'✅ First odometry received: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}, {self.current_pose[2]:.1f}°)')
        
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
        
        # Call service and wait for response
        future = self.set_pose_client.call_async(request)
        
        # Wait for the service call to complete (with timeout)
        timeout_sec = 2.0
        start_time = self.get_clock().now()
        
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                raise RuntimeError('/set_pose service call timed out')
        
        # Check result
        try:
            response = future.result()
            # SetPose service has an empty response, just check that it completed without exception
            self.get_logger().info(f'✅ Initial pose set successfully')
        except Exception as e:
            raise RuntimeError(f'/set_pose service call failed: {e}')
        
    
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
