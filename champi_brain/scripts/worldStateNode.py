import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List
from champi_brain.champi_brain.worldState import WorldState, GameElement, ElementState
import yaml
import os

class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')

        self.create_subscription(
            String,  # TODO replace with custom message type
            '/new_game_element_observation',
            self.observation_callback,
            10)
        self.publisher = self.create_publisher(String, '/world_state', 10)  # TODO Replace with custom msg

        # Load initial world state from YAML using WorldState.from_yaml
        config_path = os.path.join(os.path.dirname(__file__), '../config/world_state.yaml')
        self.world_state = WorldState.from_yaml(config_path)
        self.get_logger().info('WorldStateNode started with initial state from YAML')

    def observation_callback(self, msg):
        # TODO: Parse msg.data to get observed box (x, y)
        # Update world state with each observation
        # self.world_state.process_observation(x, y)
        # Optionally publish the updated world state
        self.publish_world_state()

    def publish_world_state(self):
        # TODO: Serialize world state to a message
        msg = String()
        msg.data = ';'.join(f"{e.id},{e.x},{e.y},{e.state.value}" for e in self.world_state.elements.values())
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
