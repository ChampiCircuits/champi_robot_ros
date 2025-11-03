#!/usr/bin/env python3
"""
ROS Action Executor - ROS2 implementation of ActionExecutor interface.
Communicates with ROS topics and action servers to control the robot.
"""

from typing import Optional
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int8, Bool
from champi_interfaces.action import Navigate
from geometry_msgs.msg import Pose
from champi_brain.strategy_dsl import MotionParams


class ROSActionExecutor:
    """
    ROS2 implementation of the ActionExecutor interface.
    
    This class handles all ROS communication for robot control:
    - Navigation via /navigate action server
    - Actuator control via /ctrl/actuators topic
    - Dynamic layer control via /use_dynamic_layer topic
    """
    
    def __init__(self, node: Node):
        """
        Initialize ROS action executor.
        
        Args:
            node: ROS2 node for creating publishers/action clients
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Action client for navigation
        self.navigate_client = ActionClient(node, Navigate, '/navigate')
        self.current_goal_handle = None
        
        # Publishers
        self.actuator_pub = node.create_publisher(Int8, '/ctrl/actuators', 10)
        self.use_dynamic_layer_pub = node.create_publisher(Bool, '/use_dynamic_layer', 10)
        
        # Wait for action server
        self.logger.info('Waiting for /navigate action server...')
        self.navigate_client.wait_for_server()
        self.logger.info('Connected to /navigate action server')
        
        # Callbacks (to be set by the state machine node)
        self.on_goal_accepted = lambda: None
        self.on_goal_rejected = lambda: None
        self.on_goal_reached = lambda: None  # Called when goal succeeds
        self.on_goal_failed = lambda msg: None  # Called when goal fails
    
    def move_to(self, x: float, y: float, theta_deg: float, motion_params: MotionParams) -> None:
        """
        Send navigation goal to the robot.
        
        Args:
            x: Target x coordinate in meters
            y: Target y coordinate in meters  
            theta_deg: Target orientation in degrees
            motion_params: Motion parameters (speed, acceleration, etc.)
        """
        self.logger.info(f'Sending move goal: ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)')
        
        # Publish dynamic layer setting
        use_dynamic_msg = Bool()
        use_dynamic_msg.data = motion_params.use_dynamic_layer
        self.use_dynamic_layer_pub.publish(use_dynamic_msg)
        
        # Create goal
        goal = self._create_navigate_goal(x, y, theta_deg, motion_params)
        
        # Send goal asynchronously
        send_goal_future = self.navigate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def detect_platform(self) -> None:
        """
        Trigger platform detection.
        Note: Detection is handled by sensor callbacks in the state machine.
        """
        self.logger.info('Platform detection triggered')
        # Platform detection is passive - sensors publish to topics
        # The state machine will process sensor data
    
    def execute_actuator_action(self, action_name: str) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            action_name: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        self.logger.info(f'Executing actuator action: {action_name}')
        
        action_id = self._action_name_to_id(action_name)
        if action_id is None:
            self.logger.error(f'Unknown actuator action: {action_name}')
            return
        
        msg = Int8()
        msg.data = action_id
        self.actuator_pub.publish(msg)
    
    def wait(self, duration: float) -> None:
        """
        Wait for a duration.
        Note: Actual waiting is handled by the state machine's wait state.
        
        Args:
            duration: Duration in seconds
        """
        self.logger.info(f'Starting wait for {duration:.1f}s')
        # Waiting is handled by the state machine timer
    
    def cancel_current_action(self) -> None:
        """Cancel currently executing navigation goal."""
        if self.current_goal_handle is not None:
            self.logger.info('Cancelling current navigation goal')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
    
    # ====================== Private Methods ======================
    
    def _create_navigate_goal(self, x: float, y: float, theta_deg: float, 
                              motion_params: MotionParams) -> Navigate.Goal:
        """Create Navigate.Goal message from parameters."""
        goal = Navigate.Goal()
        
        # Target pose
        theta_rad = theta_deg * math.pi / 180.0
        goal.pose = Pose()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal.pose.orientation.w = math.cos(theta_rad / 2.0)
        
        # Motion parameters
        goal.max_linear_speed = motion_params.speed
        goal.max_angular_speed = 3.0
        goal.accel_linear = motion_params.accel_linear
        goal.accel_angular = motion_params.accel_angular
        goal.end_speed = motion_params.end_speed
        
        # Tolerances
        goal.linear_tolerance = 0.005
        goal.angular_tolerance = 0.05
        
        # Look-at-point (disabled by default)
        goal.do_look_at_point = False
        
        # Timeout
        goal.timeout = 20.0
        
        return goal
    
    def _action_name_to_id(self, action_name: str) -> Optional[int]:
        """Convert action name to actuator ID."""
        action_map = {
            'PUT_BANNER': 0,
            'TAKE_LOWER_PLANK': 1,
            'TAKE_UPPER_PLANK': 2,
            'PUT_LOWER_PLANK_LAYER_1': 3,
            'PUT_UPPER_PLANK_LAYER_2': 4,
            'TAKE_CANS_RIGHT': 5,
            'TAKE_CANS_LEFT': 6,
            'PUT_CANS_RIGHT_LAYER_2': 7,
            'PUT_CANS_LEFT_LAYER_1': 8,
            'RESET_ACTUATORS': 9,
            'STOP_ALL_MOTORS': 10,
            'ENABLE_ALL_MOTORS': 11,
            'GET_READY': 12,
        }
        return action_map.get(action_name)
    
    # ====================== Callbacks ======================
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.warn('Navigation goal rejected')
            self.on_goal_rejected()
            return
        
        self.logger.info('Navigation goal accepted')
        self.current_goal_handle = goal_handle
        self.on_goal_accepted()
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)
    
    def _goal_result_callback(self, future):
        """Handle goal completion."""
        result = future.result().result
        self.logger.info(f'Navigation completed: success={result.success}, message={result.message}')
        
        self.current_goal_handle = None
        
        # Call appropriate callback
        if result.success:
            self.on_goal_reached()
        else:
            self.on_goal_failed(result.message)
    
    def _cancel_done_callback(self, future):
        """Handle goal cancellation."""
        self.logger.info('Navigation goal cancelled')
        self.current_goal_handle = None
