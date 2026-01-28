#!/usr/bin/env python3
"""
ROS2 node for processing watchtower camera images.
This node subscribes to Webots camera topics and processes received images.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from aruco_localizer import ArucoDetector


class WatchtowerImageProcessor(Node):
    """
    Image processing node for the watchtower surveillance camera.
    
    This node allows monitoring the entire playing field from a fixed external
    position and performs image processing on the video stream.
    """

    def __init__(self):
        super().__init__('watchtower_image_processor')
        
        # OpenCV-ROS bridge
        self.bridge = CvBridge()
        
        # Variables to store camera information
        self.camera_info = None
        self.latest_image = None
        
        # Configurable parameters
        self.declare_parameter('show_processed_image', True)
        self.declare_parameter('processing_rate_hz', 10.0)
        
        # Get parameters
        self.show_image = self.get_parameter('show_processed_image').get_parameter_value().bool_value
        self.processing_rate = self.get_parameter('processing_rate_hz').get_parameter_value().double_value
        
        # Subscribe to camera topics
        self.image_subscription = self.create_subscription(
            Image,
            '/watchtower/camera/image_color',
            self.image_callback,
            10
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/watchtower/camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.arucos_poses_pub = self.create_publisher(ArucoPoses, '/watchtower/aruco_poses', 10)

        # Timer for periodic processing
        self.processing_timer = self.create_timer(
            1.0 / self.processing_rate,
            self.process_latest_image
        )
        self.detector = ArucoDetector()
        
        # Counters for statistics
        self.frame_count = 0
        self.processed_count = 0
        
        self.get_logger().info('Watchtower Image Processor started')
        self.get_logger().info(f'Show images: {self.show_image}')


    def camera_info_callback(self, msg):
        """
        Callback for camera calibration information.
        
        Args:
            msg (CameraInfo): Message containing camera parameters
        """
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'Camera info received: {msg.width}x{msg.height}')
            self.get_logger().info(f'Distortion model: {msg.distortion_model}')
            self.get_logger().info(f'Camera matrix: {msg.k}')
            
    def image_callback(self, msg):
        """
        Callback for camera images.
        Stores the latest received image for later processing.
        
        Args:
            msg (Image): ROS2 image message
        """
        try:
            # ROS -> OpenCV conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def process_latest_image(self):
        """
        Processes the latest received image according to configured frequency.
        This function is called by the processing timer.
        """
        if self.latest_image is None:
            return
        
        try:
            processed_image = self.process_image(self.latest_image.copy())
            self.processed_count += 1
            
            # Display images if enabled
            if self.show_image:
                self.display_images(self.latest_image, processed_image)
            
            # Periodic statistics
            if self.processed_count % 100 == 0:
                self.get_logger().info(f'Images processed: {self.processed_count}/{self.frame_count}')
                
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')


    def process_image(self, image):
        """
        MAIN IMAGE PROCESSING FUNCTION
        
        This is where you'll implement your computer vision algorithms!
        
        Args:
            image (np.ndarray): OpenCV image in BGR format
            
        Returns:
            np.ndarray: Processed image
        """
        
        # === BASIC PROCESSING EXAMPLE ===
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        poses, ids = self.detector.detect_arucos()
        message = ArucoPoses()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_frame"
        for pose, marker_id in zip(poses, ids):
            marker = ArucoPose()
            marker.id = int(marker_id)
            marker.pose = pose
            message.markers.append(marker)

        

    def display_images(self, original, processed):
        """
        Displays original and processed images side by side.
        
        Args:
            original (np.ndarray): Original image
            processed (np.ndarray): Processed image
        """
        # Resize if images are too large
        height, width = original.shape[:2]
        if width > 640:
            scale = 640 / width
            new_width = int(width * scale)
            new_height = int(height * scale)
            original = cv2.resize(original, (new_width, new_height))
            processed = cv2.resize(processed, (new_width, new_height))
        
        # Horizontal concatenation
        combined = np.hstack((original, processed))
        
        # Add separator
        cv2.line(combined, (original.shape[1], 0), (original.shape[1], original.shape[0]), 
                (255, 255, 255), 2)
        
        # Add labels
        cv2.putText(combined, 'ORIGINAL', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined, 'PROCESSED', (original.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add help text
        help_text = "Press 'q' or ESC to exit"
        cv2.putText(combined, help_text, (10, combined.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('Watchtower', combined)
        

        #####################################################################################################################
        #####################################################################################################################
        #####################################################################################################################

        # Check for keyboard input and window close
        key = cv2.waitKey(1) & 0xFF
        
        # Check if window is closed (X button clicked)
        if cv2.getWindowProperty('Watchtower', cv2.WND_PROP_VISIBLE) < 1:
            self.get_logger().info("Window closed by user, shutting down...")
            rclpy.shutdown()
            return
            
        # Allow closing with 'q' or 'ESC' key
        if key == ord('q') or key == 27:  # 27 is ESC key
            self.get_logger().info("Exit key pressed, shutting down...")
            rclpy.shutdown()
            return


    def destroy_node(self):
        """Cleanup when closing the node."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WatchtowerImageProcessor()
        
        node.get_logger().info("Watchtower Image Processor node started. Ctrl+C to stop.")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
