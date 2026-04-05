#!/usr/bin/env python3
"""
Action Executor Interface - Interface for executing robot actions.
This allows the state machine to be independent of ROS implementation.
"""

from math import radians, sin, cos
from geometry_msgs.msg import Pose
from typing import Protocol, Optional
from rclpy.action import ActionClient
from champi_brain.strategy_dsl import MotionParams
from champi_interfaces.action import Navigate
from std_msgs.msg import Int8, Bool
from rclpy.node import Node
from champi_brain.actuator_commands import ActuatorCommand, actuator_name_to_id
from abc import abstractmethod

class ActionExecutor():
    """
    Interface with abstract methods for executing robot actions.

    This defines the contract that any executor must implement.
    Allows for different implementations (ROS, Simulation, Mock for testing).
    """
    def __init__(self, node: Node):
        """
        Initialize action executor.

        """
        
        self.node = node
        self.logger = node.get_logger()
        
        # Action client for navigation
        self.navigate_client = ActionClient(node, Navigate, '/navigate')
        self.current_goal_handle = None
        
        # Publishers
        self.actuator_pub = node.create_publisher(Int8, '/ctrl/actuators', 10)
        
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
        self.logger.info(f'Sending move goal: ({x:.2f}, {y:.2f}, {theta_deg:.1f}°) with {motion_params}')
        
        # Create goal
        goal = self._create_navigate_goal(x, y, theta_deg, motion_params)
        
        # Send goal asynchronously
        send_goal_future = self.navigate_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    @abstractmethod
    def detect_platform(self) -> None:
        """
        Trigger platform detection using sensors.
        The detected position will be made available through callbacks.
        """
        ...
        # TODO delete ?
    
    @abstractmethod
    def execute_actuator_action(self, action_name: str) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            action_name: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        ...

    def wait(self, duration: float) -> None:
        """
        Wait for a duration.
        Note: Actual waiting is handled by the state machine's wait state.
        
        Args:
            duration: Duration in seconds
        """
        self.logger.info(f'Starting wait for {duration:.1f}s')
        # Waiting is handled by the state machine timer # TODO ??
        
    
    def cancel_current_action(self) -> None:
        """Cancel currently executing navigation goal."""
        if self.current_goal_handle is not None:
            self.logger.warn('Cancelling current navigation goal')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
    
    # ====================== Private Methods ======================
    
    def _create_navigate_goal(self, x: float, y: float, theta_deg: float, 
                              motion_params: MotionParams) -> Navigate.Goal:
        """Create Navigate.Goal message from parameters."""
        goal = Navigate.Goal()
        
        # Target pose
        theta_rad = radians(theta_deg)
        goal.pose = Pose()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = sin(theta_rad / 2.0)
        goal.pose.orientation.w = cos(theta_rad / 2.0)
        
        # Motion parameters
        goal.max_linear_speed = motion_params.speed
        goal.max_angular_speed = 3.0
        goal.accel_linear = motion_params.accel_linear
        goal.accel_angular = motion_params.accel_angular
        goal.end_speed = motion_params.end_speed
        # TODO send use_collision_avoidance to goal

        
        # Tolerances
        goal.linear_tolerance = 0.005
        goal.angular_tolerance = 0.05
        
        # Look-at-point (disabled by default)
        goal.do_look_at_point = False
        
        # Timeout
        goal.timeout = 20.0
        
        return goal

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
