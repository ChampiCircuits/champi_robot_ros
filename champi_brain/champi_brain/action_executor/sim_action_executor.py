#!/usr/bin/env python3
"""
SIM Action Executor - Simulation implementation of ActionExecutor interface.
Simulates robot actions without real hardware.
"""

import time
from rclpy.node import Node
from champi_brain.action_executor.action_executor import ActionExecutor
from std_msgs.msg import Int8

class SIMActionExecutor(ActionExecutor):
    """
    Simulation implementation of the ActionExecutor interface.
    """
    
    def __init__(self, node: Node, simulate_actuators_delays: bool) -> None:
        """
        Initialize SIM action executor.
        
        Args:
            node: ROS2 node for creating publishers/action clients
        """

        super().__init__(node)
        self.simulate_actuators_delays = simulate_actuators_delays
    
    def execute_actuator_action(self, action_name: str) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            action_name: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        if not self.simulate_actuators_delays:
            self.logger.info(f'Executing actuator action: {action_name} in sim, no delay simulated')
            return
        
        delay = 2.0  # seconds, simulate actuator action duration
        self.logger.info(f'Executing actuator action: {action_name} in sim, so waiting {delay}s')
        time.sleep(delay) # TODO non blocking or not ??
        # TODO temps variable selon l'action, a def dans un fichier config comme les points
        self.logger.info(f'Executing actuator action: {action_name} done !')

    def detect_platform(self) -> None:
        """
        Trigger platform detection using sensors.
        The detected position will be made available through callbacks.
        """
        ...
        # TODO delete or return a fixed value ?