#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from champi_interfaces.action import Navigate
from rclpy.logging import get_logger

from rclpy.executors import ExternalShutdownException
import math


class PathPlannerUINode(Node):

    def __init__(self):
        super().__init__('planner_ui')
        self.get_logger().info(f"\tLaunching Path planner UI...")


        # Subscriber for /goal_pose
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Action client for /navigate
        self.action_client_navigate = ActionClient(self, Navigate, '/navigate')
        self.action_client_navigate.wait_for_server()
        self.get_logger().info('Action client for /navigate is ready!')

        self.goal_handle_navigate = None

        self.goal_pose = None
        self.get_logger().info(f"\tPath planner UI launched !")
    

    # ==================================== ROS2 Callbacks ==========================================

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'New goal received!')

        self.goal_pose = msg.pose

        # Cancel current if self.future_navigate_result not None
        if self.goal_handle_navigate is not None:
            self.get_logger().info('Cancelling current goal...')

            future = self.goal_handle_navigate.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
        else:
            self.send_goal(self.goal_pose)
        


    def feedback_callback(self, feedback_msg):
        pass
        # self.get_logger().info(f'Feedback received! path_compute_result:{self.path_compute_result_to_str(feedback_msg.feedback.path_compute_result)}, ETA: {feedback_msg.feedback.eta}')



    # ==================================== Done Callbacks ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.goal_handle_navigate = goal_handle

        self.get_logger().info('Goal accepted!')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action result: {result.success}, {result.message}')


    def cancel_done_callback(self, future):
        self.get_logger().info('Goal cancelled succesfully!')
        self.send_goal(self.goal_pose)


# ============================================ Utils ==============================================


    def send_goal(self, goal_pose):
        # Create a Navigate request
        goal = self.create_action_goal(goal_pose)

        # Send the goal to the action server
        future_navigate_result = self.action_client_navigate.send_goal_async(goal, feedback_callback=self.feedback_callback)

        # Add done callback
        future_navigate_result.add_done_callback(self.goal_response_callback)

        self.get_logger().info('Goal sent...')


    def create_action_goal(self, goal_pose):

        goal = Navigate.Goal()
        
        goal.pose = goal_pose
        
        goal.end_speed = 0.

        goal.max_linear_speed = 0.5
        goal.max_angular_speed = math.pi/2

        goal.linear_tolerance = 0.005
        goal.angular_tolerance = 0.05

        goal.do_look_at_point = False

        goal.look_at_point = Point()
        goal.look_at_point.x = 1.
        goal.look_at_point.y = 1.
        goal.look_at_point.z = 0.

        goal.robot_angle_when_looking_at_point = 0.

        goal.timeout = 10. # seconds

        return goal
    

    def path_compute_result_to_str(self, path_compute_result):
        if path_compute_result == Navigate.Feedback.SUCCESS_STRAIGHT:
            return 'SUCCESS_STRAIGHT'
        elif path_compute_result == Navigate.Feedback.SUCCESS_AVOIDANCE:
            return 'SUCCESS_AVOIDANCE'
        elif path_compute_result == Navigate.Feedback.START_NOT_IN_COSTMAP:
            return 'START_NOT_IN_COSTMAP'
        elif path_compute_result == Navigate.Feedback.GOAL_NOT_IN_COSTMAP:
            return 'GOAL_NOT_IN_COSTMAP'
        elif path_compute_result == Navigate.Feedback.GOAL_IN_OCCUPIED_CELL:
            return 'GOAL_IN_OCCUPIED_CELL'
        elif path_compute_result == Navigate.Feedback.NO_PATH_FOUND:
            return 'NO_PATH_FOUND'
        elif path_compute_result == Navigate.Feedback.INTITIALIZING:
            return 'INTITIALIZING'
        else:
            return 'UNKNOWN (error)'

 


def main(args=None):
    rclpy.init(args=args)

    node = PathPlannerUINode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().warn("Path Planner UI terminated")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
