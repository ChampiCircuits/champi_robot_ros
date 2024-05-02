#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi

from rclpy.executors import ExternalShutdownException

class TestGoal(Node):

    def __init__(self):

        super().__init__('test_goal_node')
    
        # pub pose stamped
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goals = [[1.0, 1.0, 0.], [1.5, 1.5, 0.]]
        # self.goals = [[0., 1., -pi/2], [1., 1., 0.], [1., 0., pi/2],[0., 0., 0.]]
        self.i_goal = 0

        timer_period = 25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.timer_callback()



    def timer_callback(self):

        goal = PoseStamped()
        # set timestamp 
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goals[self.i_goal][0]
        goal.pose.position.y = self.goals[self.i_goal][1]
        goal.pose.position.z = 0.
        goal.pose.orientation.x = 0.
        goal.pose.orientation.y = 0.
        goal.pose.orientation.z = sin(self.goals[self.i_goal][2]/2)
        goal.pose.orientation.w = cos(self.goals[self.i_goal][2]/2)

        self.pub.publish(goal)

        print("goal:",self.goals[self.i_goal])

        self.i_goal += 1
        if self.i_goal == len(self.goals):
            self.i_goal = 0

    # destructor
    def __del__(self):
        print('destroying TestGoal')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TestGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()