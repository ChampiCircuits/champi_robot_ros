#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

from icecream import ic
from math import pi, atan2
import numpy as np

from champi_navigation.gui_v2 import GuiV2
from champi_navigation.pose_control import PoseControl, Vel


class PoseControlNode(Node):

    def __init__(self):
        super().__init__('pose_control_node')

        # Parameters
        self.control_loop_period = self.declare_parameter('control_loop_period', 0.1).value
        self.enable_viz = self.declare_parameter('viz', True).value

        # Viz in Rviz
        if self.enable_viz:
            self.viz = GuiV2(self)
        else:
            self.viz = None

        self.poseControl = PoseControl(self.viz)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_sub  # prevent unused variable warning
        
        # Timers
        self.timer = self.create_timer(self.control_loop_period, self.control_loop_callback)

        # TF related
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Diagnostic
        self.last_time_ctrl_loop = None


    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        theta = 2*atan2(q.z, q.w)
        self.poseControl.set_goal(np.array([x, y, theta]))


    def control_loop_callback(self):

        # Initialize the last_time_ctrl_loop
        if self.last_time_ctrl_loop is None:
            self.last_time_ctrl_loop = self.get_clock().now()
            return
        
        # Compute the time elapsed since the last control loop
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_ctrl_loop).nanoseconds / 1e9
        self.last_time_ctrl_loop = current_time

        # Transmit the current state of the robot to the pose control
        current_robot_pose = self.get_robot_pose_from_tf()
        if current_robot_pose is None: # TF could not be received
            return
        current_robot_speed = self.get_current_robot_speed()
        self.poseControl.robot_state.update(current_robot_pose, current_robot_speed)

        # Call the control loop of the pose control
        cmd_twist = self.poseControl.control_loop_spin_once(dt)

        cmd_twist_stamped = TwistStamped()
        cmd_twist_stamped.header.stamp = current_time.to_msg()
        cmd_twist_stamped.header.frame_id = "base_link"
        cmd_twist_stamped.twist = cmd_twist

        # Publish the command
        self.cmd_vel_pub.publish(cmd_twist_stamped)


    def get_robot_pose_from_tf(self):
        t = None
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"odom"} to {"base_link"}: {ex}')
            return None
        
        q = t.transform.rotation
        pose = np.array([t.transform.translation.x, t.transform.translation.y, 2*atan2(q.z, q.w)])

        # Conversion entre -pi et pi
        if pose[2] > pi:
            pose[2] -= 2*pi
        elif pose[2] < -pi:
            pose[2] += 2*pi
        
        return pose


    def get_current_robot_speed(self):
        return Vel(0, 0, 0) # TODO


def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PoseControlNode()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()