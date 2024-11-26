#!/usr/bin/env python3

import champi_navigation.goal_checker as goal_checker
from champi_libraries_py.utils.angles import get_yaw
from champi_libraries_py.data_types.robot_state import RobotState
from champi_libraries_py.data_types.geometry import Pose2D, Vel2D
from champi_libraries_py.utils.diagnostics import create_topic_freq_diagnostic, ExecTimeMeasurer
from champi_libraries_py.utils.timeout import Timeout
from champi_libraries_py.utils.dt_measurer import DtMeasurer
from champi_navigation.cmd_vel_updaters import CmdVelUpdaterWPILib
from champi_navigation.path_follow_params import PathFollowParams

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from champi_interfaces.msg import CtrlGoal

from rclpy.executors import ExternalShutdownException

import diagnostic_msgs
import diagnostic_updater

from icecream import ic


class PoseControllerNode(Node):
    def __init__(self):
        super().__init__('pose_controller_node')

        # Parameters
        control_loop_period = self.declare_parameter('control_loop_period', rclpy.Parameter.Type.DOUBLE).value
        max_linear_acceleration = self.declare_parameter('max_linear_acceleration', rclpy.Parameter.Type.DOUBLE).value
        max_angular_acceleration = self.declare_parameter('max_angular_acceleration', rclpy.Parameter.Type.DOUBLE).value
        self.timeout_wait_next_goal = self.declare_parameter('timeout_wait_next_goal', rclpy.Parameter.Type.DOUBLE).value

        # Print parameters
        self.get_logger().info('Path Controller started with the following parameters:')
        self.get_logger().info(f'control_loop_period: {control_loop_period}')
        self.get_logger().info(f'max_linear_acceleration: {max_linear_acceleration}')
        self.get_logger().info(f'max_angular_acceleration: {max_angular_acceleration}')

        # ROS subscriptions, publishers and timers
        self.timer = self.create_timer(control_loop_period, self.control_loop_spin_once)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.champi_path_sub = self.create_subscription(CtrlGoal, '/ctrl_goal', self.ctrl_goal_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, '/odometry/filtered', self.current_pose_callback, 10)

        # Other objects instantiation
        self.cmd_vel_updater = CmdVelUpdaterWPILib()
        self.path_follow_params = PathFollowParams(max_linear_acceleration, max_angular_acceleration)
        self.dt_measurer = DtMeasurer(control_loop_period)
        self.robot_current_state = None
        self.ctrl_goal: CtrlGoal = None

        # "stop" variable, that is used to indicate that planner asked to stop the robot (by sending a goal with max speed = 0)
        self.stop_requested_by_planner = False

        # When the ctrl_goal is reached and end_speed!=0 (we expect a new goal to be sent right away),
        # the robot will emergency stop once the timeout is elapsed.
        self.timeout = Timeout()

        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')

        # Diagnostic for cmd_vel frequency
        self.diagnostic_freq_cmd_vel = create_topic_freq_diagnostic('/cmd_vel', updater, 10)
        
        # Diagnostic for execution time of control loop
        self.exec_time_measurer = ExecTimeMeasurer()
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)

        # Diagnostic for cmd_vel updater
        updater.add('CmdVelUpdater', self.cmd_vel_updater.produce_diagnostics)
        
        # Diagnostic for current goal
        updater.add('Current goal', self.diag_current_goal)
        

    def current_pose_callback(self, msg):
        """Callback for the current pose message. It is called when a new pose is received from topic."""
        pose = Pose2D(pose=msg.pose.pose)
        vel = Vel2D(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        vel = Vel2D.to_global_frame(pose, vel)
        self.robot_current_state = RobotState(pose, vel)


    def ctrl_goal_callback(self, ctrl_goal: CtrlGoal):
        """Callback to receive ctrl_goal messages from topic."""

        if self.robot_current_state is None:
            return
        
        # Stop requested by planner ?
        if ctrl_goal.max_linear_speed == 0.:
            self.stop_requested_by_planner = True
            return
        else:
            self.stop_requested_by_planner = False
        
        if self.ctrl_goal is None or not self.are_ctrl_goals_equal(ctrl_goal, self.ctrl_goal):
            # Means that we need to update the control goal (first time or new goal)
            # Why do we test this ? The path planner is not required to publish the twice the same goal (e.g periodically)
            # but for reliability, it can if it wants to.
            self.ctrl_goal = ctrl_goal
            self.path_follow_params.update_ctrl_goal(self.robot_current_state, ctrl_goal)  # See description of PathFollowParams class for more details


            if self.timeout.is_running():
                self.timeout.reset()


    def control_loop_spin_once(self):
        """Control loop routine. Called periodically by a timer."""

        # # Exectution time measurement for diagnostics. Check class definition for details.
        self.exec_time_measurer.start()

        self.dt_measurer.tick()

        # Initialization checks or stop requested by planner
        if self.robot_current_state is None or self.ctrl_goal is None or self.stop_requested_by_planner:
            self.stop_robot()
            self.exec_time_measurer.stop()
            return

        # When goal is reached, we allow a timeout before stopping the robot, to allow some time to the path planner or brain to send a new goal.

        # In case end_speed is non zero:
        #   We enter this block once, when the goal is reached. Note that in the next iterations, is_ctrl_goal_reached can return False
        #   because the robot may overtake tne goal.
        # In case end_speed is zero:
        #   We enter this block every time the goal is reached, because the robot will stop thus is_ctrl_goal_reached will stay True.
        if not self.timeout.is_running() and goal_checker.is_ctrl_goal_reached(self.ctrl_goal, self.robot_current_state.pose):  # Goal reached
            if self.ctrl_goal.end_speed != 0.:
                self.timeout.start(self.timeout_wait_next_goal)
            else:
                self.stop_robot()
                self.exec_time_measurer.stop()
                return

        # One the timeout is elapsed, we stop the robot
        if self.timeout.is_running():
            if self.timeout.is_elapsed():
                self.stop_robot()
            self.exec_time_measurer.stop()
            return

        # Compute the velocity command
        self.path_follow_params.update_robot_state(self.robot_current_state)
        dt = self.dt_measurer.get_dt()
        cmd_vel = self.cmd_vel_updater.compute_cmd_vel(dt, self.path_follow_params)

        # express cmd in the base_link frame (initially in the global fixed frame)
        cmd_vel = Vel2D.to_robot_frame(self.robot_current_state.pose, cmd_vel)
        # convert to cmd_twist
        cmd_twist = cmd_vel.to_twist()

        # Publish the command
        self.cmd_vel_pub.publish(cmd_twist)
        self.diagnostic_freq_cmd_vel.tick()

        self.exec_time_measurer.stop()
    

    def are_ctrl_goals_equal(self, goal1: CtrlGoal, goal2: CtrlGoal):
        """
        Check if two control goals are equal, position-wise.
        """
        return goal1.pose.position.x == goal2.pose.position.x and \
                goal1.pose.position.y == goal2.pose.position.y


    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.linear.y = 0.
        cmd_vel.angular.z = 0.
        self.cmd_vel_pub.publish(cmd_vel)
        self.diagnostic_freq_cmd_vel.tick()
    

    def diag_current_goal(self, stat):
        """Callback for the diagnostic updater to report the current goal."""

        if self.ctrl_goal is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'No goal received yet')
            stat.add('Goal x (m)', 'N/A')
            stat.add('Goal y (m)', 'N/A')
            stat.add('Goal yaw (rad)', 'N/A')
            return stat
        
        stat.add('Goal x (m)', str(self.ctrl_goal.pose.position.x))
        stat.add('Goal y (m)', str(self.ctrl_goal.pose.position.y))
        stat.add('Goal yaw (rad)', str(get_yaw(self.ctrl_goal.pose)))

        if goal_checker.is_ctrl_goal_reached(self.ctrl_goal, self.robot_current_state.pose) or self.timeout.is_running():
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Goal reached')
        elif self.stop_requested_by_planner:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Stop requested by planner')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Going to goal')

        return stat


def main(args=None):
    rclpy.init(args=args)

    node = PoseControllerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()