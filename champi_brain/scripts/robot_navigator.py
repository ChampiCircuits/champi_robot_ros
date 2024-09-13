#!/usr/bin/env python3

from champi_interfaces.msg import ChampiPath, ChampiSegment, ChampiPoint


from typing import Tuple
from math import sin, cos
from std_msgs.msg import Empty
from rclpy.action import ActionClient
from rclpy.logging import get_logger
import utils

from geometry_msgs.msg import Pose
from champi_interfaces.action import Navigate

from math import sin, cos

from rclpy.logging import get_logger

class Robot_Navigator():
    def __init__(self, node):
        get_logger('robot_navigator').info(f"\tLaunching Robot Navigator...")
        
        self.node = node

        # Action client for /navigate
        self.action_client_navigate = ActionClient(self.node, Navigate, '/navigate')
        get_logger('robot_navigator').info('\t\tWaiting for Action Server /navigate...')
        self.action_client_navigate.wait_for_server()
        get_logger('robot_navigator').info('\t\tAction client for /navigate is ready!')

        self.gfini_pub = self.node.create_publisher(Empty, '/gfini', 10)


        self.goal_handle_navigate = None

        self.goal_pose = None
        self.last_goal = None
        get_logger('robot_navigator').info(f"\tRobot Navigator launched !")


    def navigate_through_waypoints(self, waypoints: list[Pose|None], max_time_allowed: float):
        navigate_goal = Navigate.Goal() # = ChampiPath type
        navigate_goal.path.name = ""
        navigate_goal.path.max_time_allowed = float(max_time_allowed)

        waypoints.insert(0, None) #to be later replaced by current_pose in path_planner_node

        for i in range(len(waypoints)):
            if i+1 == len(waypoints):
                break

            segment = ChampiSegment()
            segment.name = ""
            if waypoints[i] != None:
                segment.start = utils.pose_to_champi_point(waypoints[i])
            segment.end = utils.pose_to_champi_point(waypoints[i+1])
            if i == 0:
                segment.max_linear_speed = 1.0
            else:
                segment.max_linear_speed = 0.01
            segment.max_angular_speed = 1.5
            # segment.do_look_at_point = 
            # segment.look_at_point = 
            # segment.robot_angle_when_looking_at_point = 

            navigate_goal.path.segments.append(segment)

        # Send the goal to the action server
        future_navigate_result = self.action_client_navigate.send_goal_async(navigate_goal, feedback_callback=self.feedback_callback)

        # Add done callback
        future_navigate_result.add_done_callback(self.goal_response_callback)

        get_logger('robot_navigator').info('Waypoints sent from robot_navigator!')


    def tuple_to_pose(self, tuple: Tuple[float, float, float]):
        pose = Pose()
        pose.position.x = tuple[0]
        pose.position.y = tuple[1]
        pose.orientation.z = sin(tuple[2]/2)
        pose.orientation.w = cos(tuple[2]/2)
        return pose
    
    def navigate_to_tuple(self, destination: Tuple[float, float, float], max_time_allowed:float): # TODO change calls to Pose msg
        if destination is None:
            return
        if self.last_goal is not None and self.last_goal[0] == destination[0] and self.last_goal[1] == destination[1] and self.last_goal[2] == destination[2]:
            return
        self.last_goal = destination

        pose = self.tuple_to_pose(destination)

        self.navigate_through_waypoints([pose], max_time_allowed)

    # ==================================== ROS2 Callbacks ==========================================

    def feedback_callback(self, feedback_msg):
        get_logger('robot_navigator').info(f'Feedback received! Feedback: {feedback_msg}')


    # ==================================== Done Callbacks ==========================================

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            get_logger('robot_navigator').info('Goal rejected :(')
            return
        
        self.goal_handle_navigate = goal_handle

        get_logger('robot_navigator').info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        

    def get_result_callback(self, future):
        get_logger('robot_navigator').error(f'Received a result !')
        result = future.result().result
        get_logger('robot_navigator').warn(f'Result: {result.success}, {result.message}')
        if result.success:
            msg = Empty()
            self.gfini_pub.publish(msg)
        # else:
        #     print(result.)

    def cancel_done_callback(self, future):
        get_logger('robot_navigator').info('Goal cancelled succesfully!')
        self.navigate_through_waypoints([self.goal_pose], 100000.) # TODO WHY??

