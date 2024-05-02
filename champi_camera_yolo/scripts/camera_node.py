#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import champi_camera_yolo.pyueye_api as pyueye_api
import numpy as np
from rclpy.executors import ExternalShutdownException


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.get_logger().info('camera node initializing... !')

        self.publisher_ = self.create_publisher(Image, "/camera_images", 10)  # Le topic où les images seront publiées
        self.timer = self.create_timer(1.0, self.capture_and_publish)  # Capture toutes les 100 ms
        self.bridge = CvBridge()
        self.get_logger().info('camera node initialized !')


    def capture_and_publish(self):
        # Appel de la fonction get_image
        frame = pyueye_api.get_image() # 8UC4 images
        frame = np.array(frame)
        print(frame)

        # Convertit l'image OpenCV en message ROS
        # ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgra8")#bgra8?
        # # # Publie l'image sur le topic
        # self.publisher_.publish(ros_image)
        # self.get_logger().info('published !')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    node.get_logger().info('camera node started ! ')
    


    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n\n\nPASS")
        pass
    finally:
        print("\n\n\nFINALLY")
        pyueye_api.release()
        node.destroy_node()
        rclpy.try_shutdown()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()
