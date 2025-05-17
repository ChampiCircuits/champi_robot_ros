#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KayaControlNode(Node):
    def __init__(self):
        super().__init__('kaya_control_node')
        
        # Publisher to send velocity commands to the Kaya robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to control the rate of publishing
        self.timer = self.create_timer(1.0, self.move_kaya)  # Update every second

        # Define movement sequence
        self.movement_sequence = [
            'forward',
            'rotate_left',
            'backward',
            'rotate_right'
        ]
        self.current_index = 0
        self.sequence_duration = 5.0  # Duration for each movement in seconds
        self.start_time = self.get_clock().now()

        # Define velocities for different movements
        self.velocity = Twist()

    def move_kaya(self):
        # Determine the elapsed time for current movement
        current_time = self.get_clock().now().to_msg().sec
        elapsed_time = current_time - self.start_time.to_msg().sec

        # Check if it's time to switch to the next movement in the sequence
        if elapsed_time > self.sequence_duration:
            self.current_index = (self.current_index + 1) % len(self.movement_sequence)
            self.start_time = self.get_clock().now()

        # Set velocity based on the current movement in the sequence
        self.set_movement(self.movement_sequence[self.current_index])
        # self.set_movement("forward")

        # Publish the velocity command
        self.cmd_vel_publisher.publish(self.velocity)
        self.get_logger().info(f'Movement State: {self.movement_sequence[self.current_index]} | Publishing: x={self.velocity.linear.x}, y={self.velocity.linear.y}, z={self.velocity.angular.z}')

    def set_movement(self, state):
        # Set the velocity based on the movement state
        if state == 'forward':
            self.velocity.linear.x = 0.8
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.0
        elif state == 'backward':
            self.velocity.linear.x = -0.8
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.0
        elif state == 'rotate_left':
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.2
        elif state == 'rotate_right':
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = -0.2
        else:
            # Default to no movement if state is unknown
            self.velocity.linear.x = 0.0
            self.velocity.linear.y = 0.0
            self.velocity.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = KayaControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()