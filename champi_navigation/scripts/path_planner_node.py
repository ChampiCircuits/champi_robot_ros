#!/usr/bin/env python3
import math

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from champi_interfaces.action import Navigate
from champi_interfaces.msg import CtrlGoal
from geometry_msgs.msg import Pose, PoseStamped, Twist

from math import hypot
import numpy as np
import time
from threading import Lock


from champi_navigation.path_planner import PathPlanner, ComputePathResult
from champi_navigation.planning_feedback import ComputePathResult, get_feedback_msg
import champi_navigation.goal_checker as goal_checker
from champi_libraries_py.utils.diagnostics import ExecTimeMeasurer
from champi_libraries_py.utils.timeout import Timeout
from champi_libraries_py.data_types.geometry import Pose2D, Vect2D
from champi_libraries_py.utils.angles import get_yaw

import diagnostic_msgs
import diagnostic_updater

from icecream import ic



from rclpy.logging import get_logger

from icecream import ic


class PlannerNode(Node):
    """
    Path planner node. It has an action server. It receives Navigate goals and computes a path to reach them.
    Then it publishes the command to the controller node as a CtrlGoal.
    """

    def __init__(self):
        super().__init__('planner_node')
        #self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Parameters
        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_tolerance = self.declare_parameter('waypoint_tolerance', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_speed_linear = self.declare_parameter('waypoint_speed_linear', rclpy.Parameter.Type.DOUBLE).value
        self.debug = self.declare_parameter('debug', rclpy.Parameter.Type.BOOL).value
        self.enemy_detect_dist = self.declare_parameter('enemy_detect_dist', rclpy.Parameter.Type.DOUBLE).value
        self.enemy_detect_angle = self.declare_parameter('enemy_detect_angle', rclpy.Parameter.Type.DOUBLE).value
        self.backoff_distance = self.declare_parameter('backoff_distance', rclpy.Parameter.Type.DOUBLE).value

        # Print parameters
        self.get_logger().info('Path Planner started with the following parameters:')
        self.get_logger().info(f'loop_period: {self.loop_period}')
        self.get_logger().info(f'waypoint_tolerance: {self.waypoint_tolerance}')
        self.get_logger().info(f'debug: {self.debug}')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.enemy_odom_sub = self.create_subscription(Odometry, '/enemy_pose', self.enemy_odom_callback, 10)
        self.latest_enemy_pose = Pose2D(x=-100., y=-100.)

        # Publisher
        self.champi_path_pub = self.create_publisher(CtrlGoal, '/ctrl_goal', 10)
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)

        self.cmd_vel_stop_pub = self.create_publisher(Twist, '/emergency/cmd_vel_stop', 10)

        # Action Server /navigate
        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.navigate_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Path Planner started /navigate server')

        # Timeout object to abort the Navigate goal if it takes too long (robot stuck, goal occupied...)
        self.timeout = Timeout()

        # Diagnostic updater
        updater = diagnostic_updater.Updater(self)
        updater.setHardwareID('none')

        # Diagnostic for execution time of control loop
        self.exec_time_measurer = ExecTimeMeasurer()
        updater.add('Loop exec time', self.exec_time_measurer.produce_diagnostics)

        # Diagnostic for timeout
        updater.add('Timeout', self.timeout.produce_diagnostics) 

        # Data retreived from topics
        self.robot_pose: Pose = None

        # Current goal handle, that we get when a new goal is received by the action server
        self.goal_handle_navigate = None
        
        # True when we have a goal to reach. Come back to False when the goal is reached or cancelled
        self.planning = False

        # This mutex is used to avoid the execute_callback to be running multiple times concurentlly, which leads to very annoying things.
        # For example, without mutex, when we cancel a goal, the new execute_callback is called althrough the previous one is still running,
        # and they modify the same attributes...
        self.mutex_exec = Lock()


    # ==================================== ROS2 topics Callbacks ==========================================


    def odom_callback(self, msg):
        self.robot_pose = Pose2D(pose=msg.pose.pose)

    def enemy_odom_callback(self, msg):
        self.latest_enemy_pose = Pose2D(pose=msg.pose.pose)

    def is_enemy_close(self, robot_pose: Pose2D, enemy_pose: Pose2D, goal: Pose2D):

        vect_robot = Vect2D(pose2d=robot_pose)
        vect_enemy = Vect2D(pose2d=enemy_pose)
        vect_goal = Vect2D(pose2d=goal)

        vect_robot_to_goal = vect_goal.sub(vect_robot)
        vect_robot_to_enemy = vect_enemy.sub(vect_robot)

        try:
            angle_enemy = vect_robot_to_goal.angle(vect_robot_to_enemy)
        except:
            ic(vect_robot_to_goal, vect_robot_to_enemy)
            return False # We're spinning on one point

        in_fov = abs(angle_enemy) < self.enemy_detect_angle * (math.pi/180.)

        too_close = vect_robot.sub(vect_enemy).norm() < self.enemy_detect_dist

        return in_fov and too_close


    def make_backoff_pose(self, start: Pose2D, goal: Pose2D, robot_pose: Pose2D, backoff_dist):

        vect_start = Vect2D(pose2d=start) # TODO we must not go further than the start.
        vect_goal = Vect2D(pose2d=goal)
        vect_robot = Vect2D(pose2d=robot_pose)

        vect_dir = vect_goal.sub(vect_robot).normalize()

        vect_backoff_pose = vect_robot.add(vect_dir.mult(-backoff_dist))

        backoff_pose = vect_backoff_pose.to_pose2d()
        backoff_pose.theta = goal.theta

        return backoff_pose


    def publish_emergency_stop(self):
        # Publish a Twist with zero linear and angular speeds
        twist = Twist()
        self.cmd_vel_stop_pub.publish(twist)



    # ==================================== Action Server Callbacks ==========================================


    def navigate_callback(self, navigate_goal: Navigate.Goal):
        """ Called when a new Navigate goal is received
        """
        self.get_logger().info('New Navigate request received to pose: '
                                f'({navigate_goal.pose.position.x}, {navigate_goal.pose.position.y}, {get_yaw(navigate_goal.pose)*180.0/3.14159})')

        # Store the goal
        self.current_navigate_goal = navigate_goal

        # Cancel the current goal if there is one
        if self.planning:
            self.goal_handle_navigate.abort()

            self.get_logger().info('Received a new Navigate request, cancelling the current one!')
        
        self.planning = True

        return GoalResponse.ACCEPT
    

    def cancel_callback(self, goal_handle):
        """ Called when the goal is cancelled by the client
        """
        self.get_logger().debug('Goal cancelled!')
        if self.planning:
            self.goal_handle_navigate.abort()
        else:
            self.get_logger().warn('No goal to cancel!')  # Can happen if the robot reach the end at the same time as the goal is cancelled
        self.planning = False
        return CancelResponse.ACCEPT

    
    async def execute_callback(self, goal_handle):
        """ Called right after we return GoalResponse.ACCEPT in navigate_callback
        """
        self.mutex_exec.acquire()

        self.goal_handle_navigate = goal_handle


        # =================================== INITIALIZATION ==========================================

        # We're still waiting for first messages (init)
        while rclpy.ok() and self.robot_pose is None and goal_handle.is_active:
            self.get_logger().info(f'Waiting for init messages, robot_pose={self.robot_pose is not None}', throttle_duration_sec=1.)
            
            # Feedback
            feedback_msg = get_feedback_msg(ComputePathResult.INITIALIZING, [], 0)
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(self.loop_period)


        # =================================== MAIN LOOP ==========================================

        navigate_goal_reached = False
        self.timeout.start(self.current_navigate_goal.timeout)

        start_pose = self.robot_pose

        poses_to_go = [Pose2D(pose=self.current_navigate_goal.pose)]

        while rclpy.ok() and goal_handle.is_active and not navigate_goal_reached:

            # Exectution time measurement for diagnostics. Check class definition for details.
            self.exec_time_measurer.start()

            t_loop_start = time.time()

            # Check if the timeout is reached
            if self.timeout.is_elapsed():
                self.get_logger().warn('Timeout reached!')
                goal_handle.abort()
                self.timeout.reset()
                self.exec_time_measurer.stop()
                continue

            # Check if enemy close
            # len(poses_to_go) == 1 means we are not performing backoff
            if len(poses_to_go) == 1 and self.is_enemy_close(self.robot_pose, self.latest_enemy_pose, poses_to_go[0]):

                self.get_logger().info("enemy close", throttle_duration_sec=1.)

                self.publish_emergency_stop()

                backoff_pose = self.make_backoff_pose(start_pose, poses_to_go[0], self.robot_pose, self.backoff_distance)
                # insert it first position
                poses_to_go = [backoff_pose] + poses_to_go


            # Check if goal is reached
            if goal_checker.is_goal_reached(poses_to_go[0],
                                            self.robot_pose,
                                            self.current_navigate_goal.end_speed == 0,
                                            self.current_navigate_goal.do_look_at_point,
                                            Pose2D(point=self.current_navigate_goal.look_at_point),
                                            self.current_navigate_goal.robot_angle_when_looking_at_point,
                                            self.current_navigate_goal.linear_tolerance,
                                            self.current_navigate_goal.angular_tolerance):

                if len(poses_to_go) > 1: # Switch to next goal
                    poses_to_go.pop(0)
                else:
                    navigate_goal_reached = True
                    self.exec_time_measurer.stop()
                    continue

            # Compute path
            result = ComputePathResult.SUCCESS_STRAIGHT

            # We got a valid path, create a CtrlGoal and publish it (+ debug visualizations)
            # Create a CtrlGoal
            ctrl_goal = self.create_ctrl_goal_from_navigate_goal(self.current_navigate_goal, is_waypoint=False)
            ctrl_goal.pose = poses_to_go[0].to_ros_pose()

            # Publish goal
            self.champi_path_pub.publish(ctrl_goal)

            remaining_path = [self.robot_pose] + poses_to_go

            # Publish action feedback
            feedback_msg = get_feedback_msg(result, remaining_path, self.current_navigate_goal.max_linear_speed)
            goal_handle.publish_feedback(feedback_msg)

            # Publish path for visualization
            self.publish_path([p.to_ros_pose() for p in remaining_path])

            self.exec_time_measurer.stop()

            # Sleep to respect the loop period
            sleep_time = max(0, self.loop_period - (time.time() - t_loop_start))
            time.sleep(sleep_time)

        self.planning = False
        self.timeout.reset()
        

        # ============================ FILL ACTION RESULT ====================================

        # stop robot and clear path
        self.publish_stop()
        self.publish_path([])

        result = None
        
        # Check if the goal was aborted
        if not goal_handle.is_active:
            self.get_logger().debug('Action execution stopped because goal was aborted!!')
            result =  Navigate.Result(success=False, message='Goal aborted!')

        # Check if the node is shutting down
        elif not rclpy.ok():
            result = Navigate.Result(success=False, message='Node shutdown!')

        # The goal was reached
        else:
            goal_handle.succeed()
            self.get_logger().debug('Navigate Goal reached!')
            result = Navigate.Result(success=True, message='Goal reached!')
        
        self.mutex_exec.release()
        
        return result


    # ====================================== Utils ==========================================


    def publish_stop(self):
        """Publish a stop command to the controller node. Made by publishing an empty CtrlGoal; as max speed is 0, the robot will stop.
        """
        ctrl_goal = CtrlGoal()
        self.champi_path_pub.publish(ctrl_goal)


    def publish_path(self, path: list[Pose]):
        """Publish a path (for visualization)

        Args:
            path (list[Pose]): The path to publish
        """

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for pose in path:
            path_msg.poses.append(self.pose_to_pose_stamped(pose, 'odom'))

        self.path_publisher_viz.publish(path_msg)


    def pose_to_pose_stamped(self, pose: Pose, frame_id: str) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = pose
        return pose_stamped



    def create_ctrl_goal_from_navigate_goal(self, navigate_goal: Navigate.Goal, is_waypoint) -> CtrlGoal:

        """
        Create a CtrlGoal from a Navigate goal, transferring all the fields.
        Then, you can edit what you need afterwards.
        """

        ctrl_goal = CtrlGoal()

        ctrl_goal.pose = navigate_goal.pose

        if is_waypoint:
            ctrl_goal.end_speed = self.waypoint_speed_linear
            ctrl_goal.linear_tolerance = self.waypoint_tolerance
            ctrl_goal.max_linear_speed = self.waypoint_speed_linear
        else:
            ctrl_goal.end_speed = navigate_goal.end_speed
            ctrl_goal.linear_tolerance = navigate_goal.linear_tolerance
            ctrl_goal.max_linear_speed = navigate_goal.max_linear_speed

        ctrl_goal.max_angular_speed = navigate_goal.max_angular_speed

        ctrl_goal.accel_linear = navigate_goal.accel_linear
        ctrl_goal.accel_angular = navigate_goal.accel_angular

        ctrl_goal.angular_tolerance = navigate_goal.angular_tolerance

        ctrl_goal.do_look_at_point = navigate_goal.do_look_at_point
        ctrl_goal.look_at_point = navigate_goal.look_at_point
        ctrl_goal.robot_angle_when_looking_at_point = navigate_goal.robot_angle_when_looking_at_point

        return ctrl_goal
    

    # ====================================== Main ==========================================


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    # We use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
