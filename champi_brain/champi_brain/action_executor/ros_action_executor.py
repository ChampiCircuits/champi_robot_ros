#!/usr/bin/env python3
"""
ROS Action Executor - ROS2 implementation of ActionExecutor interface.
Communicates with ROS topics and action servers to control the robot.
"""

from rclpy.node import Node
from champi_brain.action_executor.action_executor import ActionExecutor
from std_msgs.msg import Int8

from champi_brain.actuator_commands import ActuatorCommand


class ROSActionExecutor(ActionExecutor):
    """
    ROS2 implementation of the ActionExecutor interface.
    
    This class handles all ROS communication for robot control:
    - Navigation via /navigate action server
    - Actuator control via /ctrl/actuators topic
    """
    
    def __init__(self, node: Node):
        """
        Initialize ROS action executor.
        
        Args:
            node: ROS2 node for creating publishers/action clients
        """

        super().__init__(node)
    
    _DETECT_NUTBOXES_TIMEOUT_S = 3.0

    def detect_nutboxes(self) -> None:
        """
        Trigger nutbox detection. Fires on_goal_failed if no valid detection
        arrives within _DETECT_NUTBOXES_TIMEOUT_S seconds.
        """
        self.logger.info('Started Nutbox detection timer...')
        self._detect_nutboxes_timeout_timer = self.node.create_timer(
            self._DETECT_NUTBOXES_TIMEOUT_S, self._on_detect_nutboxes_timeout
        )

    def _on_detect_nutboxes_timeout(self) -> None:
        self.cancel_detect_nutboxes_timeout()
        self.logger.warn(f'⚠️ Nutbox detection timeout ({self._DETECT_NUTBOXES_TIMEOUT_S:.0f}s) — cancelling group')
        self.on_goal_failed('detectNutBoxes timeout')

    def cancel_detect_nutboxes_timeout(self) -> None:
        timer = getattr(self, '_detect_nutboxes_timeout_timer', None)
        if timer is not None:
            timer.destroy()
            self._detect_nutboxes_timeout_timer = None
        
    
    def execute_actuator_action(self, actuator_command: ActuatorCommand) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            actuator_command: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        self.logger.info(f'Executing actuator action: {actuator_command}')

        msg = Int8()
        msg.data = int(actuator_command)
        self.actuator_pub.publish(msg)
        # TODO there should be feedback when action is done
        # also we should be able to tell if we want to wait for completion or not
    