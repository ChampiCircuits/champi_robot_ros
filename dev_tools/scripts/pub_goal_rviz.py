#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi

from icecream import ic

from rclpy.executors import ExternalShutdownException

class TestGoal(Node):

    def __init__(self):

        super().__init__('pub_goal_rviz')
    
        # pub pose stamped
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # subscribe to /goal_pose_rviz
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose_rviz',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        self.pub.publish(msg)




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