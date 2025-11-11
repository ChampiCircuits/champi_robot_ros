#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException

import sys
import os
from champi_brain.strategy_loader import load_strategy
from champi_brain.strategy_dsl import Action
from champi_libraries_py.utils.angles import rad_to_quat

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from math import sin, cos, radians

class StrategyPublisher(Node):

    def __init__(self, strategy_file):
        super().__init__('strategy_publisher')
        
        # Publishers
        self.markers_publisher = self.create_publisher(MarkerArray, '/strategy_markers', 10)

        self.get_logger().info('>> Loading strategy...')
        strategy_path = os.path.join(
            get_package_share_directory('champi_brain'), 'scripts', 'strategies', strategy_file
        )

        self.actions, self.init_pose, self.home_pose, self.wait_to_come_home_pose = load_strategy(strategy_path, "YELLOW", self.get_logger())
        self.get_logger().info(f'<< Strategy {strategy_file} loaded with YELLOW!')
        self.get_logger().info(f'   Total actions: {len(self.actions)}')

        # print strat
        self.get_logger().info(f'   Initial pose: {self.init_pose}')
        self.get_logger().info(f'   Home pose: {self.home_pose}')
        self.get_logger().info(f'   Wait to come home pose: {self.wait_to_come_home_pose}')
        
        for i, action in enumerate(self.actions):
            self.get_logger().info(f'   Action {i}: {action.action}')
            if action.target:
                self.get_logger().info(f' -> ({action.target.x:.2f}, {action.target.y:.2f}, {action.target.theta_deg:.0f}°)')
            else:
                self.get_logger().info('')

        # Timer to publish visualizations regularly
        self.publish_markers()  # Initial publish
        self.timer = self.create_timer(10.0, self.publish_markers)
    
    def get_action_color(self, action_type):
        """Return color based on action type"""
        # Check by category first
        if action_type == 'move':
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        elif action_type == 'moveForPlatform':
            return ColorRGBA(r=0.0, g=0.8, b=0.8, a=1.0)  # Cyan
        elif action_type == 'detectPlatform':
            return ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # Blue
        elif 'TAKE' in action_type:
            return ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange for all TAKE actions
        elif 'PUT' in action_type:
            return ColorRGBA(r=0.5, g=0.0, b=1.0, a=1.0)  # Purple for all PUT actions
        elif action_type == 'GET_READY':
            return ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        elif action_type == 'add_points':
            return ColorRGBA(r=1.0, g=0.84, b=0.0, a=1.0)  # Gold
        else:
            return ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Gray default
    
    def get_action_marker_type(self, action_type):
        """Return marker type based on action"""
        if action_type == 'move':
            return Marker.ARROW
        elif action_type == 'moveForPlatform':
            return Marker.ARROW
        elif action_type == 'detectPlatform':
            return Marker.SPHERE
        elif 'TAKE' in action_type:
            return Marker.CUBE
        elif 'PUT' in action_type:
            return Marker.CYLINDER
        else:
            return Marker.SPHERE
    
    def create_line_strip(self, points, marker_id):
        """Create a line strip connecting all points"""
        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "strategy_path"
        line.id = marker_id
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.005  # Line width
        line.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)  # White semi-transparent
        line.points = points
        return line

    def add_init_markers(self, line_points, marker_array):
        # Add init pose as starting point
        init_point = Point()
        init_point.x = self.init_pose[0]
        init_point.y = self.init_pose[1]
        init_point.z = 0.0
        line_points.append(init_point)
        
        # Create marker for init pose
        init_marker = Marker()
        init_marker.header.frame_id = "odom"
        init_marker.header.stamp = self.get_clock().now().to_msg()
        init_marker.ns = "strategy_actions"
        init_marker.id = 0
        init_marker.type = Marker.SPHERE
        init_marker.action = Marker.ADD
        init_marker.pose.position.x = self.init_pose[0]
        init_marker.pose.position.y = self.init_pose[1]
        init_marker.pose.position.z = 0.0
        init_marker.scale.x = 0.035
        init_marker.scale.y = 0.035
        init_marker.scale.z = 0.035
        init_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green for start
        marker_array.markers.append(init_marker)
        
        # Create text for init pose
        init_text = Marker()
        init_text.header.frame_id = "odom"
        init_text.header.stamp = self.get_clock().now().to_msg()
        init_text.ns = "strategy_labels"
        init_text.id = 0
        init_text.type = Marker.TEXT_VIEW_FACING
        init_text.action = Marker.ADD
        init_text.pose.position.x = self.init_pose[0]
        init_text.pose.position.y = self.init_pose[1]
        init_text.pose.position.z = 0.0
        init_text.scale.z = 0.02
        init_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        init_text.text = "START"
        marker_array.markers.append(init_text)

        return marker_array, line_points
    
    def add_home_markers(self, marker_array, line_points, marker_id):
                # Add home pose marker
        home_marker = Marker()
        home_marker.header.frame_id = "odom"
        home_marker.header.stamp = self.get_clock().now().to_msg()
        home_marker.ns = "strategy_actions"
        home_marker.id = marker_id
        home_marker.type = Marker.SPHERE
        home_marker.action = Marker.ADD
        home_marker.pose.position.x = self.home_pose[0]
        home_marker.pose.position.y = self.home_pose[1]
        home_marker.pose.position.z = 0.0
        home_marker.scale.x = 0.035
        home_marker.scale.y = 0.035
        home_marker.scale.z = 0.035
        home_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red for home
        marker_array.markers.append(home_marker)
        marker_id += 1
        
        # Add home text
        home_text = Marker()
        home_text.header.frame_id = "odom"
        home_text.header.stamp = self.get_clock().now().to_msg()
        home_text.ns = "strategy_labels"
        home_text.id = marker_id
        home_text.type = Marker.TEXT_VIEW_FACING
        home_text.action = Marker.ADD
        home_text.pose.position.x = self.home_pose[0]
        home_text.pose.position.y = self.home_pose[1]
        home_text.pose.position.z = 0.0
        home_text.scale.z = 0.02
        home_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        home_text.text = "HOME"
        marker_array.markers.append(home_text)
        marker_id += 1

        return marker_array, marker_id
    
    def add_wait_to_come_home_markers(self, marker_array, line_points, marker_id):
        # Add wait to come home pose marker
        wait_marker = Marker()
        wait_marker.header.frame_id = "odom"
        wait_marker.header.stamp = self.get_clock().now().to_msg()
        wait_marker.ns = "strategy_actions"
        wait_marker.id = marker_id
        wait_marker.type = Marker.SPHERE
        wait_marker.action = Marker.ADD
        wait_marker.pose.position.x = self.wait_to_come_home_pose[0]
        wait_marker.pose.position.y = self.wait_to_come_home_pose[1]
        wait_marker.pose.position.z = 0.0
        wait_marker.scale.x = 0.035
        wait_marker.scale.y = 0.035
        wait_marker.scale.z = 0.035
        wait_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue for wait to come home
        marker_array.markers.append(wait_marker)
        marker_id += 1
        
        # Add wait to come home text
        wait_text = Marker()
        wait_text.header.frame_id = "odom"
        wait_text.header.stamp = self.get_clock().now().to_msg()
        wait_text.ns = "strategy_labels"
        wait_text.id = marker_id
        wait_text.type = Marker.TEXT_VIEW_FACING
        wait_text.action = Marker.ADD
        wait_text.pose.position.x = self.wait_to_come_home_pose[0]
        wait_text.pose.position.y = self.wait_to_come_home_pose[1]
        wait_text.pose.position.z = 0.0
        wait_text.scale.z = 0.02
        wait_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        wait_text.text = "WAIT_TO_COME_HOME"
        marker_array.markers.append(wait_text)
        marker_id += 1

        return marker_array, marker_id
    
    def publish_markers(self):
        """Publish detailed markers for all actions"""
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        line_points: list = []

        marker_array, line_points = self.add_init_markers(line_points, marker_array)

        marker_id = 1
        action_count = 0  # Counter for displayed actions only
        stack_count = 0  # Counter for stacking actions without target at same position
        last_x, last_y, last_theta = self.init_pose[0], self.init_pose[1], self.init_pose[2]
        
        for i, action in enumerate(self.actions):
            action_type = action.action
            if action_type == 'add_points' or action_type == 'GET_READY':
                continue
            
            # Determine if action has target/offset
            has_target = action.target is not None
            has_offset = action.offset is not None
            
            # Get position
            if has_target:
                # Use target as reference position
                x, y = action.target.x, action.target.y
                theta_deg = action.target.theta_deg
                
                if has_offset:
                    # Has both target (reference) and offset - display at reference, stack vertically
                    stack_count += 1
                    # add offset taking into account current orientation (matrix multiplication)
                    x_offset = action.offset.x
                    y_offset = action.offset.y
                    theta_deg_offset = action.offset.theta_deg

                    # Apply rotation and translation
                    x = (x_offset * cos(radians(theta_deg)) - y_offset * sin(radians(theta_deg))) + x
                    y = (x_offset * sin(radians(theta_deg)) + y_offset * cos(radians(theta_deg))) + y
                    theta_deg = theta_deg + theta_deg_offset

                last_x, last_y, last_theta = x, y, theta_deg
                stack_count = -1
            else:
                # No target - use last known position
                x, y, theta_deg = last_x, last_y, last_theta
                stack_count += 1
            
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            line_points.append(point)
            
            # Create marker for this action
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "strategy_actions" if 'action' in action_type else "strategy_moves"
            marker.id = marker_id
            marker.type = self.get_action_marker_type(action_type)
            marker.action = Marker.ADD
            
            # Position - stack actions without target or with offsets vertically
            marker.pose.position.x = x
            marker.pose.position.y = y
            
            if has_target and not has_offset:
                # Absolute position - normal height
                marker.pose.position.z = 0.0
                if marker.type != Marker.ARROW:
                    marker.pose.position.z = 0.03
            else:
                # Offset or no target - stack higher up
                marker.pose.position.z = 0.05 + (stack_count * 0.1)
            
            # Orientation
            angle_rad = radians(theta_deg)
            marker.pose.orientation.z = sin(angle_rad / 2)
            marker.pose.orientation.w = cos(angle_rad / 2)
            
            # Size
            if 'move' in action_type:
                marker.scale.x = 0.05  # Arrow length
                marker.scale.y = 0.015  # Arrow width
                marker.scale.z = 0.015  # Arrow height
            else:
                marker.scale.x = 0.015
                marker.scale.y = 0.015
                marker.scale.z = 0.015
            
            # Color based on action type
            marker.color = self.get_action_color(action_type)
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # Create text label
            text_marker = Marker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "strategy_labels_move" if 'move' in action_type else "strategy_labels_action"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            # Stack text labels vertically for actions at same position
            if has_target and not has_offset:
                text_marker.pose.position.z = 0.0
            else:
                text_marker.pose.position.z = 0.05 + (stack_count * 0.025) + 0.01
            text_marker.scale.z = 0.015
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            # Create label text with sequential numbering
            label = f"{action_count}:{action_type}"
            text_marker.text = label
            
            marker_array.markers.append(text_marker)
            marker_id += 1
            action_count += 1


        marker_array, marker_id = self.add_wait_to_come_home_markers(marker_array, line_points, marker_id)
        marker_array, marker_id = self.add_home_markers(marker_array, line_points, marker_id)

        # Create line strip connecting all positions
        if len(line_points) > 1:
            line_marker = self.create_line_strip(line_points, marker_id)
            marker_array.markers.append(line_marker)
        
        self.markers_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run champi_brain strategy_publisher_node.py <strategy_file.py>")
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
