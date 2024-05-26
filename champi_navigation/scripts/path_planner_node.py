#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import String
from champi_interfaces.action import Navigate


from math import sin, cos, atan2, hypot, pi
import numpy as np

import time
from icecream import ic

from champi_navigation.path_planner import AStarPathPlanner, ComputePathResult


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.goal_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        self.goal_handle_navigate = None

        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value

        self.robot_pose = None
        self.goal_pose = None
        self.costmap = None

        self.path_planner = None

    # ==================================== ROS2 Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        ]

    def goal_callback(self, goal):
        self.get_logger().info(f'New goal received!')

        self.goal_pose = [
            goal.poses[0].position.x,
            goal.poses[0].position.y,
            2 * atan2(goal.poses[0].orientation.z, goal.poses[0].orientation.w)
        ]

        # Cancel the current goal if there is one
        ic(self.goal_handle_navigate)
        if self.goal_handle_navigate is not None:
            self.goal_handle_navigate.abort()
            self.get_logger().info('Received a new goal, cancelling the current one!')

        return GoalResponse.ACCEPT
    

    async def execute_callback(self, goal_handle):

        self.goal_handle_navigate = goal_handle

         # We're still waiting for first messages (init)
        while rclpy.ok() and goal_handle.is_active and (self.robot_pose is None or self.costmap is None):
            self.get_logger().info('Waiting for init messages', throttle_duration_sec=1.)
            # TODO feedback
            time.sleep(self.loop_period)

        while rclpy.ok() and goal_handle.is_active and not self.is_current_goal_reached():


            t_loop_start = time.time()
            
            # Compute the path
            path_msg, result = self.path_planner.compute_path(self.robot_pose, self.goal_pose, self.costmap)

            if result != ComputePathResult.SUCCESS:
                self.get_logger().warn(f'Path computation failed: {result.name}', throttle_duration_sec=0.5)

            # Publish the path and feedback
            self.path_pub.publish(path_msg)

            # TODO feedback

            time.sleep(self.loop_period - (time.time() - t_loop_start))
        
        self.goal_handle_navigate = None
        

        if not goal_handle.is_active:
            self.get_logger().info('Action execution stopped because goal was aborted!!')
            return Navigate.Result(success=False, message='Goal aborted!')
        else:
            goal_handle.succeed()
            self.get_logger().info('Goal reached!')
            return Navigate.Result(success=True, message='Goal reached!')


    def costmap_callback(self, msg):

        if self.path_planner is None:
            self.path_planner = AStarPathPlanner(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)


    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled!')
        return CancelResponse.ACCEPT

    # ====================================== Utils ==========================================

    def is_current_goal_reached(self):
        """
        Checks if the goal is reached and switch to the next one if it is the case.
        """

        if self.robot_pose is None or self.goal_pose is None:
            return False

        error_max_lin = 0.05
        error_max_ang = 0.05

        return (abs(self.robot_pose[0] - self.goal_pose[0]) < error_max_lin
                and abs(self.robot_pose[1] - self.goal_pose[1]) < error_max_lin
                and self.check_angle(self.robot_pose[2], self.goal_pose[2], error_max_ang))

    def check_angle(self, angle1, angle2, error_max):
        # check that the angle error is less than error_max
        error = abs(angle1 - angle2)
        if abs(2 * pi - error) < 0.01:
            error = 0
        return error < error_max


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
