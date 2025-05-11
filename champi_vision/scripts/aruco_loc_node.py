#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2, math
import numpy as np
import matplotlib.pyplot as plt
import time
import os

from scipy.spatial.transform import Rotation as R

import champi_vision.bird_view as bv

from icecream import ic


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}"

class VisualLocalizationNode(Node):

    def __init__(self):
        super().__init__('visual_loc')

        # Parameters
        self.enable_cv2_viz = True
        self.enable_viz = False
        self.img_viz = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map to undistort the image
        self.map1 = None
        self.map2 = None

        # Bird view / image
        self.cv_bridge = CvBridge()
        self.curent_image = None
        self.latest_img = None
        self.time_last_image = time.time()  # for FPS
        self.pos_cam_in_bird_view_pxls = None
        self.bird_view = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters =  cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)


        self.timer = self.create_timer(0.1, self.timer_callback)  # 5Hz

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
        self.pub_odom = self.create_publisher(PoseWithCovarianceStamped, '/pose/visual_loc', 10)
        self.bird_view_RGB_publisher = self.create_publisher(Image, '/champi_bird_view_RGB', 10)
        self.pub_image = self.create_publisher(Image, '/image_viz', 10)


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

        if self.enable_cv2_viz:
            img_viz = bird_view_img.copy()
            cv2.imshow('Bird view', bird_view_img)
            cv2.waitKey(1)

        
        # ================================== DETECTION =========================================

        markerCorners, markerIds, _ = self.aruco_detector.detectMarkers(bird_view_img)

        #  if markers empty return
        if markerIds is None:
            self.get_logger().info("No marker detected", once=True)
            return
        
        marker_corner = None
        marker_id = None
        for i, markerId in enumerate(markerIds):
            if markerId >= 20 and markerId <= 23:
                marker_corner = markerCorners[i]
                marker_id = markerId
                break
        if marker_corner is None:
            self.get_logger().info("No marker detected", once=True)
            return

        # ========================= ARUCO -> ROBOT POSE ESTIMATION =====================================

        # get the corners of the marker
        corners = markerCorners[i]

        # get the center of the marker
        center_marker = np.mean(corners[0], axis=0)

        # Marker angle relative to horizontal axis (first side angle)
        dx = corners[0][1][0] - corners[0][0][0]
        dy = corners[0][1][1] - corners[0][0][1]
        angle_aruco_in_bv = np.arctan2(dy, dx)  # angle en radians

        # self.get_logger().info(f"center_marker {center_marker}, angle {angle_rad*180/3.14}°")

        pos_aruco_in_bv = np.array([center_marker[0], center_marker[1], 1])

        pos_aruco_in_bv_m = np.linalg.inv(self.bird_view.M_workplane_real_to_img_) @ pos_aruco_in_bv

        ic(pos_aruco_in_bv_m)

        # ========================= ROBOT -> WORLD POSE ESTIMATION =====================================

        if markerIds[i]==20:
            x = 2.-0.45-0.12/2.
            y = 3.-0.7-0.12/2.
        elif markerIds[i]==21:
            x = 2.-0.45-0.12/2.
            y = 0.7+0.12/2.
        elif markerIds[i]==22:
            x = 0.5+0.12/2
            y = 3.-0.7-0.12/2.
        elif markerIds[i]==23:
            x = 0.5+0.12/2
            y = 0.7+0.12/2.

        pos_aruco_in_world = np.array([x, y, 1])

        # Now we have the position of the aruco tag relative to the robot, and we know the position of the tag in the world.
        # We can compute the position of the robot in the world frame. We do this computation using pos_aruco_in_bv_m, pos_aruco_in_world and angle_aruco_in_bv

        # ic(pos_aruco_in_bv_m, pos_aruco_in_world, angle_aruco_in_bv)

        # Compute the position of the robot in the world frame
        pos_robot_in_world = pos_aruco_in_world - np.matmul(R.from_euler('z', angle_aruco_in_bv, degrees=False).as_matrix(), pos_aruco_in_bv_m)

        # ic(pos_robot_in_world)

        pose_robot = pos_robot_in_world


        # ic(pose_robot, markerIds[i])

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.pose.position.x = pose_robot[0]
        pose_msg.pose.pose.position.y = pose_robot[1]
        pose_msg.pose.pose.position.z = 0.
        pose_msg.pose.pose.orientation.x = 0.
        pose_msg.pose.pose.orientation.y = 0.
        pose_msg.pose.pose.orientation.z = np.sin(angle_aruco_in_bv/2)
        pose_msg.pose.pose.orientation.w = np.cos(angle_aruco_in_bv/2)


        self.pub_odom.publish(pose_msg)

        if self.enable_cv2_viz:
            # draw 2D axis
            img_viz = self.draw_2D_axis(img_viz, pos_aruco_in_bv[:2], angle_aruco_in_bv)
            cv2.imshow('Detections', img_viz)
            cv2.waitKey(1)


        if self.enable_viz:

            # publish image
            self.publish_imageMono(img_viz)

    def init_bird_view(self):
        transform = None
        try:
            # get transform, which is what we need to wait for
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame',rclpy.time.Time().to_msg())
            print('transform received')
            # # unsubscribe from camera info (can't do that in the callback otherwise the nodes crashes) TODO IT MAKES THE NODE CRASH
            # self.subscription_cam_info.destroy()

            # compute transform between the 2 frames as 1 matrix
            rot = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            rot = rot.as_matrix()
            trans = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

            ic(trans)

            transform_mtx = np.concatenate([rot, trans.reshape(3,1)], axis=1)
            transform_mtx = np.concatenate([transform_mtx, np.array([[0,0,0,1]])], axis=0)

            # initialize bird view
            K = np.array(self.camera_info.k).reshape(3,3)
            self.bird_view = bv.BirdView(K, transform_mtx, (0., -0.4), (1., 0.4), resolution=378)

            self.pos_cam_in_bird_view_pxls = self.bird_view.get_work_plane_pt_in_bird_img(np.array([0, 0, 1]))
            ic(self.pos_cam_in_bird_view_pxls)

            # compute undistortion map
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, np.array(self.camera_info.d), None, K, (self.camera_info.width,self.camera_info.height), cv2.CV_32FC1)

            self.get_logger().info("Node Initialized", once=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("waiting for tf2 transform...", once=True)
            return

    def callback_cam_info(self, msg):
        if self.camera_info is None:
            self.camera_info = msg

    def image_callback(self, msg):
        self.latest_img = msg

    def publish_imageRGB(self, image):
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='rgb8')
        self.bird_view_RGB_publisher.publish(image_msg)

    def publish_imageMono(self, image):
        image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='mono8')
        self.pub_image.publish(image_msg)

    def call_set_pose(self, pose):

        request = SetPose.Request()
        # request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = pose.x
        request.pose.pose.pose.position.y = pose.y
        request.pose.pose.pose.position.z = 0.
        request.pose.pose.pose.orientation.x = 0.
        request.pose.pose.pose.orientation.y = 0.
        request.pose.pose.pose.orientation.z = np.sin(pose.theta/2)
        request.pose.pose.pose.orientation.w = np.cos(pose.theta/2)

        self.get_logger().info('Calling /set_pose...')
        self.future_set_pose_response = self.set_pose_client.call_async(request)

    def draw_2D_axis(self, image, pos_pxls, angle):
        img = image.copy()
        pos_pxls = (int(pos_pxls[0]), int(pos_pxls[1]))
        cv2.circle(img, pos_pxls, 10, (255,0,0), -1)

        # axis
        axis_len = 100
        x_axis = np.array([axis_len, 0])
        y_axis = np.array([0, axis_len])
        x_axis_rot = np.matmul(np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]), x_axis)
        y_axis_rot = np.matmul(np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]), y_axis)
        x_axis_rot = x_axis_rot.astype(int)
        y_axis_rot = y_axis_rot.astype(int)
        x_axis_rot = x_axis_rot + pos_pxls
        y_axis_rot = y_axis_rot + pos_pxls
        cv2.line(img, tuple(pos_pxls), tuple(x_axis_rot), (0,255,0), 2)
        cv2.line(img, tuple(pos_pxls), tuple(y_axis_rot), (0,0,255), 2)
        return img


    def draw_fps(self, image):
        # compute fps
        fps = 1 / (time.time() - self.time_last_image)
        self.time_last_image = time.time()

        # draw fps
        cv2.putText(image, f"FPS: {fps:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


    def load_ref_image(self, path):
        image = cv2.imread(path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image


def main(args=None):
    rclpy.init(args=args)

    visual_localization = VisualLocalizationNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()