import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class VisualLocalizationNode(Node):

    def __init__(self):
        super().__init__('visual_loc')
        self.subscription = self.create_subscription(
            Image,
            '/champi/sensors/camera_1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard')


def main(args=None):
    rclpy.init(args=args)

    visual_localization = VisualLocalizationNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()