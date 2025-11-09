#!/usr/bin/env python3
"""
ROS Action Executor - ROS2 implementation of ActionExecutor interface.
Communicates with ROS topics and action servers to control the robot.
"""

from rclpy.node import Node
from champi_brain.action_executor.action_executor import ActionExecutor
from std_msgs.msg import Int8

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
    
    def detect_platform(self) -> None:
        """
        Trigger platform detection.
        Note: Detection is handled by sensor callbacks in the state machine.
        """
        self.logger.info('Platform detection triggered')
        
    
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
        # TODO there should be feedback when action is done
        # also we should be able to tell if we want to wait for completion or not
    