#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from champi_brain.world_state.worldState import WorldState, NutsBox
from champi_interfaces.msg import TableObservation, GameElement
from math import radians
from champi_libraries_py.utils.angles import rad_to_quat

class WorldStateNode(Node):
    def __init__(self):
        """
        Node that maintains and updates the world state based on observations.
        Receives observations as TableObservation messages, updates the world state,
        and publishes the updated world state.
        """
        super().__init__('world_state_node')
        self.get_logger().info(f'Initializing WorldStateNode...')

        # ============================================================
        # PARAMETERS
        # ============================================================   
        self.declare_parameter('matching_distance_threshold', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_missing', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('initial_world_state_file', rclpy.Parameter.Type.STRING)

        matching_distance_threshold = self.get_parameter('matching_distance_threshold').value
        max_missing = self.get_parameter('max_missing').value
        initial_world_state_file = self.get_parameter('initial_world_state_file').value

        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'\tmatching_distance_threshold: {matching_distance_threshold}')
        self.get_logger().info(f'\tmax_missing: {max_missing}')
        self.get_logger().info(f'\tinitial_world_state_file: {initial_world_state_file}')

        # ============================================================
        # ROS
        # ============================================================
        self.create_subscription(
            TableObservation,
            '/new_table_observation',
            self.observation_callback,
            10)
        
        # TRANSIENT_LOCAL QoS to keep last message for late subscribers
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher = self.create_publisher(TableObservation, '/world_state', latched_qos)

        # Load initial world state from YAML using WorldState.from_yaml
        config_path = get_package_share_directory('champi_brain') + '/config/' + initial_world_state_file
        init_elements, init_zones = WorldState.from_yaml(config_path)
        self.world_state = WorldState(
            elements=init_elements,
            init_zones=init_zones,
            matching_distance_threshold=matching_distance_threshold,
            max_missing=max_missing
        )
        # publish initial world state
        self.publish_world_state()
        self.get_logger().info(f'WorldStateNode started !')

    def observation_callback(self, msg):
        # Convert TableObservation message to list of NutsBox
        detections = []
        for i, elem in enumerate(msg.elements):
            detection = NutsBox(
                id=f'detection_{i}',  # Temporary ID for detections
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

    def constructTableObservationMsg(self):
        # Retrieve current world state and create a TableObservation message
        msg = TableObservation()
        for elem in self.world_state.elements.values():
            game_elem = GameElement()
            game_elem.id = elem.id
            game_elem.type = 'nut_box'  # All elements are nut boxes for now
            pose = Pose()
            pose.position.x = elem.x
            pose.position.y = elem.y
            pose.orientation.z, pose.orientation.w = rad_to_quat(radians(elem.theta_deg))
            game_elem.pose = pose
            game_elem.state = elem.state.value
            game_elem.color = elem.color.value
            msg.detected_game_elements.append(game_elem)
        return msg
        
    def publish_world_state(self):
        self.get_logger().info('Publishing current world state...')
        self.publisher.publish(self.constructTableObservationMsg())


def main(args=None):
    rclpy.init(args=args)
    node = WorldStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
