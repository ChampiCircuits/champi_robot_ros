#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robot_localization.srv import SetPose
from math import cos, sin, pi

from rclpy.executors import ExternalShutdownException

class CallSetPoseNode(Node):

    def __init__(self):

        super().__init__('call_set_pose')

        self.sub_initial_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10)
        
        # service client
        self.client = self.create_client(SetPose, '/set_pose')

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        # Call service /set_pose
        request = SetPose.Request()
        request.pose.pose = msg.pose
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        future = self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    node = CallSetPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()