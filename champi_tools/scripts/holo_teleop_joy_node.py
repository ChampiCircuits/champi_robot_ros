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

    TAKE_2_BOXES = 6
    BRING_2_BOXES_ON_TOP = 7
    PUT_2_LAST_BOXES_ON_THE_GROUND = 8

    PREPARE_TOP_PUSHER = 9
    GRAB_AND_SORT_2_BOXES_FROM_LIFT = 10
    PUSH_2_BOXES_OUT = 11
    OPEN_EXIT_RAMP = 12

"""
  RESET_ACTUATORS = SELECT

  THERMOMETER_LOWER_SERVO = L2
  THERMOMETER_RAISE_SERVO = L1

  TAKE_2_BOXES = A
  BRING_2_BOXES_ON_TOP = A + UP
  PUT_2_LAST_BOXES_ON_THE_GROUND = A + DOWN

  PREPARE_TOP_PUSHER = Y
  GRAB_AND_SORT_2_BOXES_FROM_LIFT = Y + LEFT
  PUSH_2_BOXES_OUT = X
  OPEN_EXIT_RAMP = B
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
        left = joy_msg.axes[DPadAxis.LEFT_RIGHT] == 1.0

        for i, (prev, current) in enumerate(zip(self.prev_buttons, joy_msg.buttons)):
            # Detect rising edge (button press)
            if current == 1 and prev == 0:
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

                # A Button combinations (Lift & Clamp)
                elif i == XboxButton.A:
                    if up:
                        msg.data = int(ActuatorCommand.BRING_2_BOXES_ON_TOP)
                        action = 'BRING_2_BOXES_ON_TOP'
                    elif down:
                        msg.data = int(ActuatorCommand.PUT_2_LAST_BOXES_ON_THE_GROUND)
                        action = 'PUT_2_LAST_BOXES_ON_THE_GROUND'
                    else:
                        msg.data = int(ActuatorCommand.TAKE_2_BOXES)
                        action = 'TAKE_2_BOXES'

                # Y Button combinations (Top Pusher & Sorting)
                elif i == XboxButton.Y:
                    if left:
                        msg.data = int(ActuatorCommand.GRAB_AND_SORT_2_BOXES_FROM_LIFT)
                        action = 'GRAB_AND_SORT_2_BOXES_FROM_LIFT'
                    else:
                        msg.data = int(ActuatorCommand.PREPARE_TOP_PUSHER)
                        action = 'PREPARE_TOP_PUSHER'

                # PUSH_2_BOXES_OUT = X
                elif i == XboxButton.X:
                    msg.data = int(ActuatorCommand.PUSH_2_BOXES_OUT)
                    action = 'PUSH_2_BOXES_OUT'

                # OPEN_EXIT_RAMP = B
                elif i == XboxButton.B:
                    msg.data = int(ActuatorCommand.OPEN_EXIT_RAMP)
                    action = 'OPEN_EXIT_RAMP'

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