#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException

import sys
import os
from strategies.strategy_loader import load_strategy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class StrategyPublisher(Node):

    def __init__(self, strategy_file):
        super().__init__('strategy_publisher')
        self.publisher_ = self.create_publisher(Path, '/strategy_path', 10)

        self.get_logger().info('>> Loading strategy...')
        strategy_path = os.path.join(
            get_package_share_directory('champi_brain'), 'scripts', 'strategies', strategy_file
        )

        self.path_targets, self.init_pose = load_strategy(strategy_path, self.get_logger())
        self.get_logger().info(f'<< Strategy {strategy_file} loaded!')

        # Timer to publish the path regularly
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # add init_pose
        init_pose = PoseStamped()
        init_pose.header.frame_id = "odom"
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x = self.init_pose[0]
        init_pose.pose.position.y = self.init_pose[1]
        init_pose.pose.orientation.w = 1.0  # No rotation for 2D display
        path_msg.poses.append(init_pose)

        for action in self.path_targets:
            if action['action'] != 'move':
                continue
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = action['target']['x']
            pose.pose.position.y = action['target']['y']
            pose.pose.orientation.w = 1.0  # No rotation for 2D display
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Published path with {} points'.format(len(self.path_targets)))


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> strategy_publisher_node.py <strategy_file.yaml>")
        return

    strategy_file = sys.argv[1]
    node = StrategyPublisher(strategy_file)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
