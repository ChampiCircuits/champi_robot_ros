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
    B = 1
    X = 3
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
class ActuatorCommand(IntEnum):
    RESET_ACTUATORS = 0
    STOP_ALL_MOTORS = 1
    ENABLE_ALL_MOTORS = 2
    GET_READY = 3

    THERMOMETER_LOWER_SERVO = 4
    THERMOMETER_RAISE_SERVO = 5

    LOWER_LEFT_ARM = 6
    LET_GO_ELEMENTS_LEFT_ARM = 7

    LOWER_RIGHT_ARM = 8
    LET_GO_ELEMENTS_RIGHT_ARM = 9

"""
  RESET_ACTUATORS = SELECT

  THERMOMETER_LOWER_SERVO = L2
  THERMOMETER_RAISE_SERVO = L1

    LOWER_LEFT_ARM = X + down
    LET_GO_ELEMENTS_LEFT_ARM = X + up

    LOWER_RIGHT_ARM = B + down
    LET_GO_ELEMENTS_RIGHT_ARM = B + up
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

        # Helper flags for D-Pad
        up = joy_msg.axes[DPadAxis.UP_DOWN] == 1.0
        down = joy_msg.axes[DPadAxis.UP_DOWN] == -1.0

        for i, (prev, current) in enumerate(zip(self.prev_buttons, joy_msg.buttons)):
            # Detect rising edge (button press)
            if current == 1 and prev == 0:
                button_name = XboxButton(i).name if i in XboxButton._value2member_map_ else f'button_{i}'
                self.get_logger().info(f'Button pressed: {button_name}')
                msg = Int8()
                action = None

                # RESET_ACTUATORS = SELECT
                if i == XboxButton.SELECT:
                    msg.data = int(ActuatorCommand.RESET_ACTUATORS)
                    action = 'RESET_ACTUATORS'

                # THERMOMETER_LOWER_SERVO = L2 / THERMOMETER_RAISE_SERVO = L1
                elif i == XboxButton.L2:
                    msg.data = int(ActuatorCommand.THERMOMETER_LOWER_SERVO)
                    action = 'THERMOMETER_LOWER_SERVO'
                elif i == XboxButton.L1:
                    msg.data = int(ActuatorCommand.THERMOMETER_RAISE_SERVO)
                    action = 'THERMOMETER_RAISE_SERVO'

                # X Button combinations (Left Arm)
                elif i == XboxButton.X:
                    if down:
                        msg.data = int(ActuatorCommand.LOWER_LEFT_ARM)
                        action = 'LOWER_LEFT_ARM'
                    elif up:
                        msg.data = int(ActuatorCommand.LET_GO_ELEMENTS_LEFT_ARM)
                        action = 'LET_GO_ELEMENTS_LEFT_ARM'

                # B Button combinations (Right Arm)
                elif i == XboxButton.B:
                    if down:
                        msg.data = int(ActuatorCommand.LOWER_RIGHT_ARM)
                        action = 'LOWER_RIGHT_ARM'
                    elif up:
                        msg.data = int(ActuatorCommand.LET_GO_ELEMENTS_RIGHT_ARM)
                        action = 'LET_GO_ELEMENTS_RIGHT_ARM'

                # Publish if an action was identified
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