#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class HoloTeleopJoy(Node):

    def __init__(self):
        super().__init__('holo_teleop_joy')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)

        self.latest_msg = None

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 5.0  # rad/s

    def listener_callback(self, msg):
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is not None:
            self.pub.publish(self.joy_to_twist(self.latest_msg))

    def joy_to_twist(self, joy_msg):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = joy_msg.axes[1] * self.max_linear_speed
        twist.twist.linear.y = joy_msg.axes[0] * self.max_linear_speed
        twist.twist.angular.z = joy_msg.axes[2] * self.max_angular_speed

        return twist


def main(args=None):
    rclpy.init(args=args)

    holo_teleop_joy = HoloTeleopJoy()

    rclpy.spin(holo_teleop_joy)

    holo_teleop_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
