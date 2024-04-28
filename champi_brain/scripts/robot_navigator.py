#!/usr/bin/env python3

from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from typing import Tuple

from utils import pose_from_position

class Robot_Navigator():
    def __init__(self):
        self.navigator = BasicNavigator()
      

    def navigate_to(self, destination: Tuple[float, float, float], max_time_allowed) -> bool:
        # convert position to m
        destination = (destination[0], destination[1], destination[2])

        start_time = self.navigator.get_clock().now()
        self.navigator.goToPose(pose_from_position(destination,self.navigator.get_clock().now().to_msg()))
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            time_of_arrival = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9

            # TODO
            # if the time from start_time plus the time of arrival is greater than the max_time_allowed, we cancel the task
            # if (self.navigator.get_clock().now() - start_time).nanoseconds / 1e9 + time_of_arrival > max_time_allowed:
            #     self.navigator.cancelTask()
            #     print("Time will be up!")
            #     return False

            # # if the time is up, we cancel the task
            # if (self.navigator.get_clock().now() - start_time).nanoseconds / 1e9 > max_time_allowed:
            #     self.navigator.cancelTask()
            #     print("Time is up!")
            #     return False

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return False
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return False            
        else:
            print('Goal has an invalid return status!')
            return False
