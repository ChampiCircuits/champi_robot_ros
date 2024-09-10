#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from champi_interfaces.action import Navigate
from champi_interfaces.msg import ChampiPath, ChampiSegment, ChampiPoint
from geometry_msgs.msg import Pose


from math import atan2, pi
import numpy as np
import time

from champi_navigation.path_planner import AStarPathPlanner, ComputePathResult
from rclpy.logging import get_logger

def pose_to_champi_point(pose: Pose) -> ChampiPoint: # TODO duplicated with champi_brain.utils
    champi_point = ChampiPoint()
    champi_point.name = ""
    champi_point.pose = pose
    champi_point.point_type = 1 # TODO use enum #TODO put the right thing here and next line
    champi_point.tolerance = 0.5 # TODO use enum
    champi_point.robot_should_stop_here = True
    return champi_point

class PlannerNode(Node):
    """
    SUBS :
        - odom
        - costmap
    PUB :
        - Path

    ACTION SERVER /navigate

    When action server is called with a goal:
        While goal not reached:
            Compute a path and publish it to /plan which will be followed by the path controller
    

    """

    def __init__(self):
        super().__init__('planner_node')
        get_logger('rclpy').info(f"\tLaunching Path planner NODE...")


        self.champi_path_pub = self.create_publisher(ChampiPath, '/plan', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.path_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        self.goal_handle_navigate = None

        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value

        self.robot_pose: Pose = None 
        self.asked_from_client_champi_path :ChampiPath = None
        self.costmap = None

        self.path_planner = None

        self.planning = False
        get_logger('rclpy').info("\tPath planner NODE launched!")

    # ==================================== ROS2 Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = Pose()
        self.robot_pose.position = msg.pose.pose.position
        self.robot_pose.orientation = msg.pose.pose.orientation
    

    def path_callback(self, navigate_goal:Navigate.Goal):
        self.get_logger().info('New goal received!')

        champi_path : ChampiPath = navigate_goal.path
        self.asked_from_client_champi_path = champi_path

        # Cancel the current goal if there is one
        if self.planning:
            self.goal_handle_navigate.abort()
            self.get_logger().info('Received a new path, cancelling the current one!')

        self.planning = True
        return GoalResponse.ACCEPT
    

    async def execute_callback(self, goal_handle):
        self.get_logger().info("callback")

        self.goal_handle_navigate = goal_handle

         # We're still waiting for first messages (init)
        while rclpy.ok() and self.planning and (self.robot_pose is None or self.costmap is None):
            self.get_logger().info('Waiting for init messages', throttle_duration_sec=1.)
            # TODO feedback
            time.sleep(self.loop_period)

        while rclpy.ok() and self.planning and not self.is_current_goal_reached():
            t_loop_start = time.time()
            
            # Compute the path, see Readme.md
            global_champi_path = ChampiPath()

            # modify the first point of the path to set it to current_pose, because it has not been set in robot_navigator
            self.asked_from_client_champi_path.segments[0].start = pose_to_champi_point(self.robot_pose)

            for champi_segment in self.asked_from_client_champi_path.segments:
                # for each segment, we compute the path
                local_champi_path, result = self.path_planner.compute_path(champi_segment.start.pose, champi_segment.end.pose, self.costmap)

                if result != ComputePathResult.SUCCESS:
                    self.get_logger().warn(f'Path computation failed: {result.name}', throttle_duration_sec=0.5)
                    # TODO gérer la situation

                # then we concatenate all paths in a global one
                for local_sub_champi_segment in local_champi_path.segments:
                    global_champi_path.segments.append(local_sub_champi_segment)
            global_champi_path.max_time_allowed = self.asked_from_client_champi_path.max_time_allowed


            # Publish the path and feedback
            self.champi_path_pub.publish(global_champi_path)

            # TODO feedback

            self.get_logger().info(f'.')
            time.sleep(self.loop_period - (time.time() - t_loop_start))


        # Publish a final empty path
        path = ChampiPath()
        path.header.stamp = self.get_clock().now().to_msg()
        self.champi_path_pub.publish(path)

        result = None

        if not self.planning:
            self.get_logger().info('Action execution stopped because goal was aborted!!')
            result =  Navigate.Result(success=False, message='Goal aborted!')
        else:
            goal_handle.succeed()
            self.get_logger().info('Goal reached!')
            self.get_logger().warn('')
            result = Navigate.Result(success=True, message='Goal reached!')
        
        self.planning = False
        return result


    def costmap_callback(self, msg):
        if self.path_planner is None:
            self.path_planner = AStarPathPlanner(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)


    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled!')
        self.goal_handle_navigate.abort()
        self.planning = False
        return CancelResponse.ACCEPT

    # ====================================== Utils ==========================================

    def is_current_goal_reached(self) -> bool:
        """
        Checks if the goal is reached and switch to the next one if it is the case.
        """
        if len(self.asked_from_client_champi_path.segments) == 0:
            return False

        last_goal_pose_of_global_path = self.asked_from_client_champi_path.segments[-1].end

        if self.robot_pose is None or last_goal_pose_of_global_path is None:
            return False

        error_max_lin = 0.05
        error_max_ang = 0.05

        theta_robot_pose = 2 * atan2(self.robot_pose.orientation.z, self.robot_pose.orientation.w)
        theta_current_goal_pose = 2 * atan2(last_goal_pose_of_global_path.pose.orientation.z, last_goal_pose_of_global_path.pose.orientation.w)

        return (abs(self.robot_pose.position.x - last_goal_pose_of_global_path.pose.position.x) < error_max_lin
                and abs(self.robot_pose.position.y - last_goal_pose_of_global_path.pose.position.y) < error_max_lin
                and self.check_angle(theta_robot_pose, theta_current_goal_pose, error_max_ang))

    def check_angle(self, angle1:float, angle2:float, error_max:float):
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
