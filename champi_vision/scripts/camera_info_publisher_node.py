#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):

    def __init__(self):

        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.declare_parameter("calib_yaml_path", "")
        filename = self.get_parameter("calib_yaml_path").value
        self.camera_info_msg = self.yaml_to_CameraInfo(filename)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def yaml_to_CameraInfo(self, yaml_fname):
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.k = calib_data["camera_matrix"]["data"]
        camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.r = calib_data["rectification_matrix"]["data"]
        camera_info_msg.p = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def timer_callback(self):
        self.publisher_.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()