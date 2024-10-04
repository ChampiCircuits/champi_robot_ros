#!/usr/bin/env python3

import champi_navigation.goal_checker as goal_checker
import champi_navigation.utils as cu
from champi_navigation.cmd_vel_updaters import CmdVelUpdaterWPILib

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from champi_interfaces.msg import CtrlGoal


class PathControllerNode(Node):
    def __init__(self):
        super().__init__('pose_control_node')

        # Parameters

        control_loop_period = self.declare_parameter('control_loop_period', rclpy.Parameter.Type.DOUBLE).value
        max_linear_acceleration = self.declare_parameter('max_linear_acceleration', rclpy.Parameter.Type.DOUBLE).value
        max_angular_acceleration = self.declare_parameter('max_angular_acceleration', rclpy.Parameter.Type.DOUBLE).value

        # Print parameters
        self.get_logger().info('Path Controller started with the following parameters:')
        self.get_logger().info(f'control_loop_period: {control_loop_period}')
        self.get_logger().info(f'max_linear_acceleration: {max_linear_acceleration}')
        self.get_logger().info(f'max_angular_acceleration: {max_angular_acceleration}')

        # ROS subscriptions, publishers and timers
        self.timer = self.create_timer(control_loop_period, self.control_loop_spin_once)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.champi_path_sub = self.create_subscription(CtrlGoal, '/plan', self.ctrl_goal_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, '/odometry/filtered', self.current_pose_callback, 10)

        # Other objects instantiation
        self.cmd_vel_updater = CmdVelUpdaterWPILib()

        self.path_follow_params = cu.PathFollowParams(max_linear_acceleration, max_angular_acceleration)

        self.robot_current_state = None

        self.ctrl_goal: CtrlGoal = None

    def current_pose_callback(self, msg):
        """Callback for the current pose message. It is called when a new pose is received from topic."""
        pose = cu.Pose2D(pose=msg.pose.pose)
        vel = cu.Vel2D(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        vel = cu.Vel2D.to_global_frame(pose, vel)
        self.robot_current_state = cu.RobotState(pose, vel)


    def ctrl_goal_callback(self, ctrl_goal: CtrlGoal):
        """Callback for the path message. It is called when a new path is received from topic."""

        if self.robot_current_state is None:
            return
        
        if self.ctrl_goal is None or not self.are_ctrl_goals_equal(ctrl_goal, self.ctrl_goal):
            self.ctrl_goal = ctrl_goal
            self.path_follow_params.update_ctrl_goal(self.robot_current_state, ctrl_goal)


    def control_loop_spin_once(self):

        if self.robot_current_state is None:
            return
        
        if self.ctrl_goal is None:
            return
        
        if goal_checker.is_ctrl_goal_reached(self.ctrl_goal, self.robot_current_state.pose):
            
            
            
            self.stop_robot()
            return

        self.update_path_follow_params()
        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(self.path_follow_params)

        # express cmd in the base_link frame
        cmd_vel = cu.Vel2D.to_robot_frame(self.robot_current_state.pose, cmd_vel)
        # convert to cmd_twist
        cmd_twist = cmd_vel.to_twist()

        # Publish the command
        self.cmd_vel_pub.publish(cmd_twist)
    

    def are_ctrl_goals_equal(self, goal1: CtrlGoal, goal2: CtrlGoal):
        """
        Check if two control goals are equal.
        """
        return goal1.pose.position.x == goal2.pose.position.x and \
                goal1.pose.position.y == goal2.pose.position.y

            
    def update_path_follow_params(self):
        """To be called in the control loop to update the robot state (and potentially other parameters later)"""

        self.path_follow_params.robot_state = self.robot_current_state


    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.linear.y = 0.
        cmd_vel.angular.z = 0.
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    pose_control_node = PathControllerNode()
    rclpy.spin(pose_control_node)
    pose_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()