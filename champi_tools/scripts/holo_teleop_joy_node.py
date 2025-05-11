#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Int8

from enum import IntEnum

INDICE_AXIS_X = 1
INDICE_AXIS_Y = 0
INDICE_AXIS_R = 2


class XboxButton(IntEnum):
    A = 0
    B = 1 # CANS_RIGHT
    X = 3 # CANS_LEFT
    Y = 4
    R2 = 9
    R1 = 7
    L1 = 6
    L2 = 8
    SELECT = 10
    START = 11

class DPadAxis(IntEnum):
    LEFT_RIGHT = 6
    UP_DOWN = 7
"""
  PUT_BANNER = 0, = R1
  TAKE_LOWER_PLANK = 1, = haut + A
  TAKE_UPPER_PLANK = 2, = haut + Y
  PUT_LOWER_PLANK_LAYER_1 = 3, = bas + A
  PUT_UPPER_PLANK_LAYER_2 = 4, = bas + Y
  TAKE_CANS_RIGHT = 5, = haut + B
  TAKE_CANS_LEFT = 6, = haut + X
  PUT_CANS_RIGHT_LAYER_1 = 7, = bas + B 
  PUT_CANS_LEFT_LAYER_2 = 8 = bas + X
  RESET_ACTUATORS = 9, = SELECT
"""

class HoloTeleopJoy(Node):

    def __init__(self):
        super().__init__('holo_teleop_joy')

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.latest_msg = None
        self.t_latest_nonzero = None

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.pub_stamped = self.create_publisher(TwistStamped, '/viz/cmd_vel_stamped', 10) # for visualization in rviz

        self.max_linear_speed = self.declare_parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE).value

        self.publisher = self.create_publisher(Int8, '/ctrl/actuators', 10)
        self.prev_buttons = []
        self.get_logger().info('Node initialized: listening to /joy')


    def joy_callback(self, joy_msg):
        self.latest_msg = joy_msg

        if not self.prev_buttons:
            self.prev_buttons = list(joy_msg.buttons)

        up = joy_msg.axes[DPadAxis.UP_DOWN] == 1.0
        down = joy_msg.axes[DPadAxis.UP_DOWN] == -1.0

        for i, (prev, current) in enumerate(zip(self.prev_buttons, joy_msg.buttons)):
            if current == 1 and prev == 0:
                msg = Int8()
                action = None

                # R1 → PUT_BANNER
                if i == XboxButton.R1:
                    msg.data = 0
                    action = 'PUT_BANNER (via R1)'

                # Y button combinations
                elif i == XboxButton.Y:
                    if up:
                        msg.data = 2  # TAKE_UPPER_PLANK
                        action = 'TAKE_UPPER_PLANK'
                    elif down:
                        msg.data = 4  # PUT_UPPER_PLANK_LAYER_2
                        action = 'PUT_UPPER_PLANK_LAYER_2'

                # A button combinations
                elif i == XboxButton.A:
                    if up:
                        msg.data = 1  # TAKE_LOWER_PLANK
                        action = 'TAKE_LOWER_PLANK'
                    elif down:
                        msg.data = 3  # PUT_LOWER_PLANK_LAYER_1
                        action = 'PUT_LOWER_PLANK_LAYER_1'

                # B button combinations
                elif i == XboxButton.B:
                    if up:
                        msg.data = 5  # TAKE_CANS_RIGHT
                        action = 'TAKE_CANS_RIGHT'
                    elif down:
                        msg.data = 7  # PUT_CANS_RIGHT_LAYER_1
                        action = 'PUT_CANS_RIGHT_LAYER_1'

                # X button combinations
                elif i == XboxButton.X:
                    if up:
                        msg.data = 6  # TAKE_CANS_LEFT
                        action = 'TAKE_CANS_LEFT'
                    elif down:
                        msg.data = 8  # PUT_CANS_LEFT_LAYER_2
                        action = 'PUT_CANS_LEFT_LAYER_2'

                # SELECT button for RESET_ACTUATORS
                elif i == XboxButton.SELECT:
                    msg.data = 9  # RESET_ACTUATORS
                    action = 'RESET_ACTUATORS (via SELECT)'

                if action is not None:
                    self.publisher.publish(msg)
                    self.get_logger().info(f'{action} → publishing {msg.data}')

        self.prev_buttons = list(joy_msg.buttons)


    def timer_callback(self):
        if self.latest_msg is None:
            return

        if self.latest_msg.axes[INDICE_AXIS_Y] != 0 or self.latest_msg.axes[INDICE_AXIS_X] != 0 or self.latest_msg.axes[INDICE_AXIS_R] != 0:
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
        twist.linear.x = joy_msg.axes[INDICE_AXIS_X] * self.max_linear_speed
        twist.linear.y = joy_msg.axes[INDICE_AXIS_Y] * self.max_linear_speed
        twist.angular.z = joy_msg.axes[INDICE_AXIS_R] * self.max_angular_speed

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