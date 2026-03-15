#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class IntrinsicUndistortTest(Node):
    def __init__(self) -> None:
        super().__init__("intrinsic_undistort_test")

        share_dir = get_package_share_directory("champi_vision")
        default_calib = os.path.join(share_dir, "config", "calib", "innomaker-OV9281.yaml")
        default_image = os.path.join(share_dir, "ressources", "images", "left-0010.png")

        self.declare_parameter("calib_yaml_path", default_calib)
        self.declare_parameter("image_path", default_image)

        calib_yaml_path = self.get_parameter("calib_yaml_path").value
        image_path = self.get_parameter("image_path").value

        self.run_once(calib_yaml_path, image_path)

    def _load_calibration(self, calib_yaml_path: str):
        with open(calib_yaml_path, "r", encoding="utf-8") as file_handle:
            calib_data = yaml.safe_load(file_handle)

        camera_matrix = np.array(calib_data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(calib_data["distortion_coefficients"]["data"], dtype=np.float64)
        return camera_matrix, dist_coeffs

    def run_once(self, calib_yaml_path: str, image_path: str) -> None:
        self.get_logger().info(f"Calibration file: {calib_yaml_path}")
        self.get_logger().info(f"Input image: {image_path}")

        if not os.path.isfile(calib_yaml_path):
            self.get_logger().error(f"Calibration file not found: {calib_yaml_path}")
            return

        if not os.path.isfile(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            return

        camera_matrix, dist_coeffs = self._load_calibration(calib_yaml_path)

        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return

        height, width = image.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (width, height), 1.0, (width, height)
        )
        undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)

        comparison = np.hstack((image, undistorted))
        cv2.putText(
            comparison,
            "Original",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            comparison,
            "Undistorted",
            (width + 20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        self.get_logger().info("Displaying image. Press q or ESC to close.")
        cv2.imshow("Intrinsic calibration undistort test", comparison)
        while rclpy.ok():
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break

        cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntrinsicUndistortTest()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
