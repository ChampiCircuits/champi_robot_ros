#!/usr/bin/env python3
"""
SIM Action Executor - Simulation implementation of ActionExecutor interface.
Simulates robot actions without real hardware.
"""

import time
from rclpy.node import Node
from champi_brain.action_executor.action_executor import ActionExecutor
from std_msgs.msg import Int8

from champi_brain.actuator_commands import ActuatorCommand


class SIMActionExecutor(ActionExecutor):
    """
    Simulation implementation of the ActionExecutor interface.
    """
    
    def __init__(self, node: Node, simulate_actuators_delays: bool) -> None:
        """
        Initialize SIM action executor.
        
        Args:
            node: ROS2 node for creating publishers/action clients
            simulate_actuators_delays: Whether to simulate delays for actuator actions
        """

        super().__init__(node)
        self.simulate_actuators_delays = simulate_actuators_delays
        self.time_per_action: dict = {}
    
    def execute_actuator_action(self, actuator_command: ActuatorCommand) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            actuator_command: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        if not self.simulate_actuators_delays:
            self.logger.info(f'Executing actuator action: {actuator_command.name} in sim, no delay simulated')
            return
        
        # Get delay from configuration
        try:
            delay = self.time_per_action[actuator_command.name]
        except KeyError:
            raise ValueError(f'No time_per_action entry for action: {actuator_command.name}, possible values: {list(self.time_per_action.keys())}')

        self.logger.info(f'Executing actuator action: {actuator_command.name} in sim, waiting {delay}s')
        time.sleep(delay)  # TODO: make non-blocking with timer callback?
        self.logger.info(f'Executing actuator action: {actuator_command.name} done!')

    def set_time_per_action(self, time_per_action: dict) -> None:
        """
        Update the time per action configuration.
        
        Args:
            time_per_action: Dictionary mapping action names to their execution time in seconds
        """
        self.time_per_action = time_per_action
        self.logger.info(f'Updated time per action configuration with {len(time_per_action)} entries')

    def detect_platform(self) -> None:
        """
        Trigger platform detection using sensors.
        The detected position will be made available through callbacks.
        """
        ...
        # TODO delete or return a fixed value ?