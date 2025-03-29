#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped


class GlobalVisionPublisherNode(Node):
    def __init__(self):
        super().__init__('global_vision_publisher_node')
        self.get_logger().info('Global Vision Publisher Node started')


        self.msg_platform = Bool()
        self.msg_pose = PoseWithCovarianceStamped()

        self.subscription_platform_info = self.create_subscription(
            Bool,
            'platform_detection',
            self.plateform_callback,
            10)

        self.subscription_position_info = self.create_subscription(
            Bool,
            '/pose/visual_loc',
            self.visual_loc_callback,
            10)
        

        self.publisher_platform = self.create_publisher(Bool, 'platform_detection', 10)


        self.timer = self.create_timer(0.5, self.timer_callback)  # 5Hz

    def timer_callback(self):
        msg = Bool()
        msg.data = True
        self.publisher_platform.publish(msg)
    
    def plateform_callback(self, msg):
        self.msg_platform = msg
    
    def visual_loc_callback(self, msg):
        self.msg_pose = msg


def main(args=None):
    rclpy.init(args=args)

    visual_localization = GlobalVisionPublisherNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()