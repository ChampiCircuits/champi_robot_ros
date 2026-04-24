#!/usr/bin/env python3
"""
SIM Action Executor - Simulation implementation of ActionExecutor interface.
Simulates robot actions without real hardware.
"""

from rclpy.node import Node
from champi_brain.action_executor.action_executor import ActionExecutor
from std_msgs.msg import Int8, Int8MultiArray
from geometry_msgs.msg import PoseStamped
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
        self._actuator_timer = None
        self.actuators_finished_pub = node.create_publisher(Int8MultiArray, '/actuators_finished', 10)
    
    def execute_actuator_action(self, actuator_command: ActuatorCommand) -> None:
        """
        Send actuator command to the robot.
        
        Args:
            actuator_command: Name of actuator action (PUT_BANNER, TAKE_CANS, etc.)
        """
        if not self.simulate_actuators_delays:
            self.logger.info(f'Executing actuator action: {actuator_command.name} in sim, no delay simulated')
            self._publish_actuator_finished(actuator_command)
            return

        # Get delay from configuration
        try:
            delay = self.time_per_action[actuator_command.name]
        except KeyError:
            raise ValueError(f'No time_per_action entry for action: {actuator_command.name}, possible values: {list(self.time_per_action.keys())}')

        self.logger.info(f'Executing actuator action: {actuator_command.name} in sim, waiting {delay}s')
        self._actuator_timer = self.node.create_timer(
            delay, lambda cmd=actuator_command: self._on_actuator_timer_done(cmd)
        )

    def _on_actuator_timer_done(self, actuator_command: ActuatorCommand) -> None:
        if self._actuator_timer is not None:
            self._actuator_timer.destroy()
            self._actuator_timer = None
        self.logger.info(f'Executing actuator action: {actuator_command.name} done!')
        self._publish_actuator_finished(actuator_command)

    def _publish_actuator_finished(self, actuator_command: ActuatorCommand) -> None:
        msg = Int8MultiArray()
        idx = int(actuator_command)
        msg.data = [0] * (idx + 1)
        msg.data[idx] = 2  # ActuatorState::DONE
        self.actuators_finished_pub.publish(msg)

    def set_time_per_action(self, time_per_action: dict) -> None:
        """
        Update the time per action configuration.
        
        Args:
            time_per_action: Dictionary mapping action names to their execution time in seconds
        """
        self.time_per_action = time_per_action
        self.logger.info(f'Updated time per action configuration with {len(time_per_action)} entries')

    def detect_nutboxes(self) -> None:
        """
        Simulate nutbox detection: inject a fake detection at a fixed relative position (0.3m ahead).
        """
        self.logger.info('[SIM] Simulating nutbox detection — injecting fake pose in 1s')
        self._sim_nutbox_timer = self.node.create_timer(1.0, self._sim_nutbox_detected_callback)

    def _sim_nutbox_detected_callback(self) -> None:
        if hasattr(self, '_sim_nutbox_timer') and self._sim_nutbox_timer is not None:
            self._sim_nutbox_timer.destroy()
            self._sim_nutbox_timer = None

        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = 0.30  # 30cm ahead
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0  # z != -1 means valid detection
        self.logger.info('[SIM] Nutbox fake detection published')
        if not hasattr(self, '_sim_nutbox_pub'):
            self._sim_nutbox_pub = self.node.create_publisher(PoseStamped, '/nutboxes_relative_position', 10)
        self._sim_nutbox_pub.publish(msg)