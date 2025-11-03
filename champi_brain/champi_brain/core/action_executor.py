#!/usr/bin/env python3
"""
Action Executor Interface - Abstract interface for executing robot actions.
This allows the state machine to be independent of ROS implementation.
"""

from typing import Protocol
from champi_brain.strategy_dsl import MotionParams


class ActionExecutor(Protocol):
    """
    Protocol (interface) for executing robot actions.
    
    This defines the contract that any executor must implement.
    Allows for different implementations (ROS, Simulation, Mock for testing).
    """
    
    def move_to(self, x: float, y: float, theta_deg: float, motion_params: MotionParams) -> None:
        """
        Command the robot to move to a specific pose.
        
        Args:
            x: Target x coordinate in meters
            y: Target y coordinate in meters
            theta_deg: Target orientation in degrees
            motion_params: Motion parameters (speed, acceleration, etc.)
        """
        ...
    
    def detect_platform(self) -> None:
        """
        Trigger platform detection using sensors.
        The detected position will be made available through callbacks.
        """
        ...
    
    def execute_actuator_action(self, action_name: str) -> None:
        """
        Execute an actuator action (e.g., grabbing, placing objects).
        
        Args:
            action_name: Name of the action to execute (PUT_BANNER, TAKE_CANS, etc.)
        """
        ...
    
    def wait(self, duration: float) -> None:
        """
        Wait for a specified duration.
        
        Args:
            duration: Duration to wait in seconds
        """
        ...
    
    def cancel_current_action(self) -> None:
        """
        Cancel the currently executing action (if any).
        Useful for emergency stops or timeout handling.
        """
        ...
