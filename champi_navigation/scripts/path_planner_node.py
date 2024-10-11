#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from champi_interfaces.action import Navigate
from champi_interfaces.msg import CtrlGoal
from geometry_msgs.msg import Pose, PoseStamped

from math import pi
import numpy as np
import time
from rclpy.executors import ExternalShutdownException

from champi_navigation.path_planner import AStarPathPlanner, ComputePathResult
import champi_navigation.goal_checker as goal_checker
from champi_libraries_py.data_types.geometry import Pose2D


from rclpy.logging import get_logger


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


        self.champi_path_pub = self.create_publisher(CtrlGoal, '/ctrl_goal', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        self.action_server_navigate = ActionServer(self, Navigate, '/navigate',
                                                   self.execute_callback,
                                                   goal_callback=self.navigate_callback,
                                                   cancel_callback=self.cancel_callback,
                                                   callback_group=ReentrantCallbackGroup())
        
        self.path_publisher_viz = self.create_publisher(Path, '/plan_viz', 10)

        self.goal_handle_navigate = None

        self.loop_period = self.declare_parameter('planner_loop_period', rclpy.Parameter.Type.DOUBLE).value
        self.waypoint_tolerance = self.declare_parameter('waypoint_tolerance', rclpy.Parameter.Type.DOUBLE).value

        self.robot_pose: Pose = None
        self.costmap = None
        self.path_planner = None

        self.planning = False
        
        get_logger('rclpy').info("\tPath planner NODE launched!")

    # ==================================== ROS2 Callbacks ==========================================

    def odom_callback(self, msg):
        self.robot_pose = Pose()
        self.robot_pose.position = msg.pose.pose.position
        self.robot_pose.orientation = msg.pose.pose.orientation
        

    def navigate_callback(self, navigate_goal: Navigate.Goal):
        self.get_logger().info('New Navigate request received!')

        # Store the goal
        self.current_navigate_goal = navigate_goal

        # Cancel the current goal if there is one
        if self.planning:
            self.goal_handle_navigate.abort()

            self.get_logger().info('Received a new Navigate request, cancelling the current one!')
        
        self.planning = True

        return GoalResponse.ACCEPT
    

    async def execute_callback(self, goal_handle):
        self.goal_handle_navigate = goal_handle

         # We're still waiting for first messages (init)
        while rclpy.ok() and (self.robot_pose is None or self.costmap is None) and goal_handle.is_active:
            self.get_logger().info('Waiting for init messages', throttle_duration_sec=1.)
            # TODO feedback
            time.sleep(self.loop_period)

        navigate_goal_reached = False

        while rclpy.ok() and goal_handle.is_active and not navigate_goal_reached:
            t_loop_start = time.time()


            # Check if goal is reached
            if goal_checker.is_goal_reached(Pose2D(pose=self.current_navigate_goal.pose),
                                            Pose2D(pose=self.robot_pose),
                                            self.current_navigate_goal.end_speed == 0,
                                            self.current_navigate_goal.do_look_at_point,
                                            Pose2D(point=self.current_navigate_goal.look_at_point),
                                            self.current_navigate_goal.robot_angle_when_looking_at_point,
                                            self.current_navigate_goal.linear_tolerance,
                                            self.current_navigate_goal.angular_tolerance):
                navigate_goal_reached = True
            

            path, result = self.path_planner.compute_path(self.robot_pose, self.current_navigate_goal.pose, self.costmap)
            
            
            if result == ComputePathResult.SUCCESS_STRAIGHT or result == ComputePathResult.SUCCESS_AVOIDANCE:

                is_waypoint = (result == ComputePathResult.SUCCESS_AVOIDANCE)

                # Create a CtrlGoal
                ctrl_goal = self.create_ctrl_goal_from_navigate_goal(self.current_navigate_goal, is_waypoint)
                ctrl_goal.pose = path[1]
                
                # Publish goal
                self.champi_path_pub.publish(ctrl_goal)

                # Publish path for visualization
                self.publish_path(path)
                

            else:
                self.get_logger().warn(f'Path computation failed: {result.name}', throttle_duration_sec=0.5)
                # TODO gérer la situation



            # TODO feedback

            # self.get_logger().info(f'. \t\t loop_exec_time={round(time.time() - t_loop_start, 5)}')
            time.sleep(self.loop_period - (time.time() - t_loop_start))


        self.planning = False

        # Publish a final empty CtrlGoal
        path = CtrlGoal()
        self.champi_path_pub.publish(path)
        
        result = None

        if not goal_handle.is_active:
            self.get_logger().info('Action execution stopped because goal was aborted!!')
            result =  Navigate.Result(success=False, message='Goal aborted!')

        elif not rclpy.ok():
            result = Navigate.Result(success=False, message='Node shutdown!')

        else:
            goal_handle.succeed()
            self.get_logger().info('Navigate Goal reached!')
            result = Navigate.Result(success=True, message='Goal reached!')
        
        return result


    def costmap_callback(self, msg):
        if self.path_planner is None:
            self.path_planner = AStarPathPlanner(msg.info.width, msg.info.height, msg.info.resolution)

        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)


    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled!')
        if self.planning:
            self.goal_handle_navigate.abort()
        else:
            self.get_logger().warn('No goal to cancel!')  # Can happen if the robot reach the end at the same time as the goal is cancelled
        self.planning = False
        return CancelResponse.ACCEPT

    # ====================================== Utils ==========================================


    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for pose in path:
            path_msg.poses.append(self.pose_to_pose_stamped(pose, 'map'))

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
            ctrl_goal.end_speed = navigate_goal.max_linear_speed
            ctrl_goal.linear_tolerance = self.waypoint_tolerance
        else:
            ctrl_goal.end_speed = navigate_goal.end_speed
            ctrl_goal.linear_tolerance = navigate_goal.linear_tolerance

        ctrl_goal.max_linear_speed = navigate_goal.max_linear_speed
        ctrl_goal.max_angular_speed = navigate_goal.max_angular_speed

        ctrl_goal.angular_tolerance = navigate_goal.angular_tolerance

        ctrl_goal.do_look_at_point = navigate_goal.do_look_at_point
        ctrl_goal.look_at_point = navigate_goal.look_at_point
        ctrl_goal.robot_angle_when_looking_at_point = navigate_goal.robot_angle_when_looking_at_point

        return ctrl_goal


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
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
