#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_localization.srv import SetPose
from math import cos, sin, pi

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

        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
        
        # service client
        self.client = self.create_client(SetPose, '/set_pose')

    def listener_callback(self, msg):
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        self.pub.publish(msg)
    
    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        # Call service /set_pose
        request = SetPose.Request()
        request.pose.pose = msg.pose
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        future = self.client.call_async(request)




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