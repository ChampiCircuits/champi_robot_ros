import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from champi_brain.world_state.worldState import WorldState, NutsBox
from champi_interfaces.msg import TableObservation, GameElement

class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')
        self.get_logger().info(f'Initializing WorldStateNode...')

        ## Retrieve parameters
        self.declare_parameter('matching_distance_threshold', 0.3)
        self.declare_parameter('max_missing', 2)
        self.declare_parameter('initial_world_state_file', 'initial_world_state.yaml')
        matching_distance_threshold = self.get_parameter('matching_distance_threshold').get_parameter_value().double_value
        max_missing = self.get_parameter('max_missing').get_parameter_value().integer_value
        initial_world_state_file = self.get_parameter('initial_world_state_file').get_parameter_value().string_value

        ## Subscriber and publisher
        self.create_subscription(
            TableObservation,
            '/new_table_observation',
            self.observation_callback,
            10)
        self.publisher = self.create_publisher(TableObservation, '/world_state', 10)

        # Load initial world state from YAML using WorldState.from_yaml
        config_path = get_package_share_directory('champi_brain') + '/config/' + initial_world_state_file
        init_elements, init_zones = WorldState.from_yaml(config_path)
        self.world_state = WorldState(
            elements=init_elements,
            init_zones=init_zones,
            matching_distance_threshold=matching_distance_threshold,
            max_missing=max_missing
        )
        self.get_logger().info(f'WorldStateNode started with initial state from YAML: {config_path}')

    def observation_callback(self, msg):
        # Convert TableObservation message to list of NutsBox
        detections = []
        for elem in msg.elements:
            detection = NutsBox(
                id=elem.id,
                x=elem.x,
                y=elem.y,
                theta_deg=elem.orientation,
                state=elem.state,
                color=elem.color
            )
            detections.append(detection)
        self.world_state.process_observation(detections)
        self.get_logger().info('World state updated with new observations.')
        self.publish_world_state()

    def publish_world_state(self):
        self.get_logger().info('Publishing current world state...')
        # Retrieve current world state and publish as TableObservation message
        msg = TableObservation()
        for elem in self.world_state.elements.values():
            game_elem = GameElement()
            game_elem.id = elem.id
            game_elem.x = elem.x
            game_elem.y = elem.y
            game_elem.orientation = elem.theta_deg
            game_elem.state = elem.state
            game_elem.color = elem.color
            msg.elements.append(game_elem)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
