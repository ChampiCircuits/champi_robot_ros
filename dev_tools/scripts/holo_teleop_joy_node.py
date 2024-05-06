#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped

from rclpy.executors import ExternalShutdownException


class HoloTeleopJoy(Node):

    def __init__(self):
        super().__init__('holo_teleop_joy')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)

        self.latest_msg = None
        self.t_latest_nonzero = None

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_stamped = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10) # for visualization in rviz

        self.max_linear_speed = 0.2  # m/s
        self.max_angular_speed = 1.5  # rad/s

        self.id_steering = None # it changes between my 2 PCs :( 2 or 3 -> we check which one is at 0 at startup
        # (the other one is the trigger so it's non-zero when not pressed)

    def listener_callback(self, msg):
        self.latest_msg = msg

        if self.id_steering is None:
            if(msg.axes[2] == 0):
                self.id_steering = 2
            else:
                self.id_steering = 3

    def timer_callback(self):
        if self.latest_msg is None:
            return

        if self.latest_msg.axes[0] != 0 or self.latest_msg.axes[1] != 0 or self.latest_msg.axes[self.id_steering] != 0:
            self.t_latest_nonzero = self.get_clock().now()

        if self.t_latest_nonzero is None:
            return

        if self.t_latest_nonzero is not None and (self.get_clock().now() - self.t_latest_nonzero).nanoseconds > 0.2e9:
            self.latest_msg = None
            return

        self.pub.publish(self.joy_to_twist(self.latest_msg))
        self.pub_stamped.publish(self.joy_to_twist_stamped(self.latest_msg))

    def joy_to_twist(self, joy_msg):
        twist = Twist()
        twist.linear.x = joy_msg.axes[1] * self.max_linear_speed
        twist.linear.y = joy_msg.axes[0] * self.max_linear_speed
        twist.angular.z = joy_msg.axes[self.id_steering] * self.max_angular_speed

        return twist

    def joy_to_twist_stamped(self, joy_msg):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist = self.joy_to_twist(joy_msg)

        return twist


def main(args=None):
    rclpy.init(args=args)

    node = HoloTeleopJoy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()