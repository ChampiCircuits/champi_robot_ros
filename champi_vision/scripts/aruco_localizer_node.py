#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from cv_bridge import CvBridge


import cv2
import numpy as np
import os

from scipy.spatial.transform import Rotation

import champi_vision.bird_view as bv
from champi_vision.aruco_localizer import ArucoDetector, Visualizer

from icecream import ic



class ArucoLocalizerNode(Node):

    def __init__(self):
        super().__init__('aruco_localizer')

        # Parameters
        self.enable_cv2_viz = False
        self.enable_topic_viz = True

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map to undistort the image
        self.map1 = None
        self.map2 = None

        # Bird view / image
        self.cv_bridge = CvBridge()
        self.curent_image = None
        self.latest_img = None
        self.bird_view = None

        self.timestamp = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Subscribe to /camera_info and /image_raw---------------------------------------------------

        # handle camera info
        self.camera_info = None
        self.subscription_cam_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.callback_cam_info,
            10)

        # handle image
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.image_subscriber  # prevent unused variable warning

        # Publisher for the visual localization and the image visualization
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/aruco_loc/pose', 10)
        self.publisher_viz = self.create_publisher(Image, '/viz/image_detection', 10)

        self.aruco_detector = ArucoDetector()
        self.visualizer = Visualizer()


    def init_bird_view(self):
        transform = None
        try:
            # get transform, which is what we need to wait for
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame',rclpy.time.Time().to_msg())
            print('transform received')
            # # unsubscribe from camera info (can't do that in the callback otherwise the nodes crashes) TODO IT MAKES THE NODE CRASH
            # self.subscription_cam_info.destroy()

            # compute transform between the 2 frames as 1 matrix
            rot = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            rot = rot.as_matrix()
            trans = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

            transform_mtx = np.concatenate([rot, trans.reshape(3,1)], axis=1)
            transform_mtx = np.concatenate([transform_mtx, np.array([[0,0,0,1]])], axis=0)

            # initialize bird view
            K = np.array(self.camera_info.k).reshape(3,3)
            self.bird_view = bv.BirdView(K, transform_mtx, (0.2, -0.4), (0.9, 0.4), resolution=378)

            # compute undistortion map
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, np.array(self.camera_info.d), None, K, (self.camera_info.width,self.camera_info.height), cv2.CV_32FC1)

            self.get_logger().info("Node Initialized", once=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("waiting for tf2 transform...", once=True)
            return


    def timer_callback(self):
        if self.latest_img is None:
            return

        # get camera info if not already
        if self.camera_info is None:
            self.get_logger().info("waiting for camera info...", once=True)
            return
        self.get_logger().info("camera info received", once=True)


        # initialize bird view if not already
        if self.bird_view is None:
            self.init_bird_view()

        if self.map1 is None:
            return

        # ==================================== PREPROCESSING ====================================
        # convert to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # undistort
        cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        self.curent_image = cv_image

        # =================================== COMPUTE BIRD VIEW ===================================

        # get bird view
        bird_view_img = self.bird_view.project_img_to_bird(self.curent_image)
        
        frame = cv2.resize(bird_view_img, (200, 140))

        new_image = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                    cv2.THRESH_BINARY, 11, 2)

        # Check if the frame is blurry
        _, blurry, variance = self.is_blurry(new_image, threshold=1000.0)

        if blurry:
            self.get_logger().warn(f"Image is blurry, variance: {variance:.2f}")
            return

        # ================================== DETECTION =========================================

        poses_markers, ids_markers = self.aruco_detector.detect_arucos(bird_view_img, ids_to_find=[20, 21, 22, 23])
        if len(poses_markers) == 0:
            self.do_viz(bird_view_img, False)
            return
        
        # get the first marker
        pose_aruco_in_bv = poses_markers[0]
        id_marker = ids_markers[0]
        self.get_logger().debug(f"tag detected: {id_marker}")

        # ========================= ARUCO -> ROBOT POSE ESTIMATION =====================================

        pos_aruco_in_bv = np.array([pose_aruco_in_bv[0], pose_aruco_in_bv[1], 1])
        pos_aruco_in_bv_m = np.linalg.inv(self.bird_view.M_workplane_real_to_img_) @ pos_aruco_in_bv

        # ========================= ROBOT -> WORLD POSE ESTIMATION =====================================

        if id_marker==20:
            x = 0.6
            y = 1.4
        elif id_marker==21:
            x = 2.4
            y = 1.4
        elif id_marker==22:
            x = 0.6
            y = 0.6
        elif id_marker==23:
            x = 2.4
            y = 0.6
        pos_aruco_in_world = np.array([x, y, 1])

        # Now we have the position of the aruco tag relative to the robot, and we know the position of the tag in the world.
        # We can compute the position of the robot in the world frame. We do this computation using pos_aruco_in_bv_m, pos_aruco_in_world and angle_aruco_in_bv

        angle_robot = pose_aruco_in_bv[2] + np.pi/2
        rot_mat_robot = Rotation.from_euler('z', angle_robot, degrees=False).as_matrix()
        pos_robot_in_world = pos_aruco_in_world[:2] - rot_mat_robot[:2, :2] @ pos_aruco_in_bv_m[:2]
        pose_robot = [pos_robot_in_world[0], pos_robot_in_world[1], angle_robot]

        self.publish_pose(pose_robot)
        self.do_viz(bird_view_img, True, pose_aruco_in_bv)

    
    def publish_pose(self, pose: list):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.
        msg.pose.pose.orientation.x = 0.
        msg.pose.pose.orientation.y = 0.
        msg.pose.pose.orientation.z = np.sin(pose[2]/2)
        msg.pose.pose.orientation.w = np.cos(pose[2]/2)
        msg.header.stamp = self.timestamp
        self.pub_pose.publish(msg)


    def callback_cam_info(self, msg):
        if self.camera_info is None:
            self.camera_info = msg


    def image_callback(self, msg):
        self.latest_img = msg
        self.timestamp = msg.header.stamp

    def is_blurry(self, image, threshold=1000.0):
        """
        Detect if an image is blurry using the Laplacian variance method.

        Args:
            image (numpy.ndarray): The input image.
            threshold (float): Variance threshold below which the image is considered blurry.

        Returns:
            bool: True if the image is blurry, False otherwise.
            float: The variance of the Laplacian.
        """

        # Compute the Laplacian of the image
        laplacian = cv2.Laplacian(image, cv2.CV_64F)

        # Compute the variance of the Laplacian
        variance = laplacian.var()

        # Determine if the image is blurry
        return laplacian, variance < threshold, variance
    def do_viz(self, image_source, detect_success: bool, detected_pose: list[float, float, float] = None): 

        if not self.enable_cv2_viz and self.curent_image is None:
            return
        
        img_viz = self.visualizer.make_viz(image_source, detect_success, detected_pose)

        if self.enable_cv2_viz:
            cv2.imshow('Detections', img_viz)
            cv2.waitKey(1)

        if self.enable_topic_viz:
            image_msg = self.cv_bridge.cv2_to_imgmsg(img_viz, encoding='rgb8')
            self.publisher_viz.publish(image_msg)




def main(args=None):
    rclpy.init(args=args)

    node = ArucoLocalizerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()