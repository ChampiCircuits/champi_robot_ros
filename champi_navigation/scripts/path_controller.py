#!/usr/bin/env python3

from champi_navigation.utils import Vel, RobotState, PathFollowParams
from champi_navigation.cmd_vel_updaters import CmdVelUpdaterWPILib
from champi_navigation.path_helper import PathHelper


from math import atan2


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from champi_interfaces.msg import ChampiPath
from std_msgs.msg import Empty


class PathControllerNode(Node):
    def __init__(self):
        super().__init__('pose_control_node')

        # Parameters

        self.control_loop_period = self.declare_parameter('control_loop_period', rclpy.Parameter.Type.DOUBLE).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE).value
        self.max_linear_acceleration = self.declare_parameter('max_linear_acceleration', rclpy.Parameter.Type.DOUBLE).value
        self.max_angular_acceleration = self.declare_parameter('max_angular_acceleration', rclpy.Parameter.Type.DOUBLE).value

        # Print parameters
        self.get_logger().info('Path Controller started with the following parameters:')
        self.get_logger().info(f'control_loop_period: {self.control_loop_period}')
        self.get_logger().info(f'max_linear_speed: {self.max_linear_speed}')
        self.get_logger().info(f'max_angular_speed: {self.max_angular_speed}')
        self.get_logger().info(f'max_linear_acceleration: {self.max_linear_acceleration}')
        self.get_logger().info(f'max_angular_acceleration: {self.max_angular_acceleration}')

        # ROS subscriptions, publishers and timers
        self.timer = self.create_timer(self.control_loop_period, self.control_loop_spin_once)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.champi_path_sub = self.create_subscription(ChampiPath, '/plan', self.champi_path_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, '/odometry/filtered', self.current_pose_callback, 10)
        self.path_finished_pub = self.create_publisher(Empty, '/path_finished', 10)

        # Other objects instantiation
        self.cmd_vel_updater = CmdVelUpdaterWPILib()

        self.path_helper = PathHelper(
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_linear_acceleration,
            self.max_angular_acceleration
        )

        self.robot_current_state = None

    def current_pose_callback(self, msg):
        """Callback for the current pose message. It is called when a new pose is received from topic."""
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)]
        vel = Vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        vel = Vel.to_global_frame(pose, vel)
        self.robot_current_state = RobotState(pose, vel)

    def champi_path_callback(self, champi_path: ChampiPath):
        """Callback for the path message. It is called when a new path is received from topic."""

        if self.robot_current_state is None:
            return
        
        if len(champi_path.segments) == 0:
            self.path_helper.set_path(champi_path)
            return
        
        self.path_helper.set_path(champi_path)

    def control_loop_spin_once(self):
        if self.path_helper.path_finished_FLAG:
            self.path_helper.path_finished_FLAG = False           
            self.path_finished_pub.publish(Empty())
            self.get_logger().warn("FINISHED IN CONTROL")

        if self.robot_current_state is None:
            return

        if self.path_helper.robot_should_stop():
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.
            cmd_vel.linear.y = 0.
            cmd_vel.angular.z = 0.
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        self.path_helper.update_goal(self.robot_current_state)

        # Maybe, when calling update_goal, we found that the robot is already at the goal
        if self.path_helper.robot_should_stop():
            return

        # Compute the command velocity (get required parameters from path_helper)
        path_follow_params = self.path_helper.get_path_follow_params(self.robot_current_state)

        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(path_follow_params)

        # express cmd in the base_link frame
        cmd_vel = Vel.to_robot_frame(self.robot_current_state.pose, cmd_vel)
        # convert to cmd_twist
        cmd_twist = cmd_vel.to_twist()

        # Publish the command
        self.cmd_vel_pub.publish(cmd_twist)


def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PathControllerNode()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()