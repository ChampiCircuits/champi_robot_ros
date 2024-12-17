#!/usr/bin/env python3
import rclpy
import tf2_ros
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

from icecream import ic
from scipy.spatial.transform import Rotation as R
import champi_vision.bird_view as bv

class BoardPlateformeDetectionNode(Node):
    def __init__(self):
        super().__init__('board_platform_detection_node')

        self.timer = self.create_timer(0.1, self.timer_callback)

        # transformation matrix between base_link and camera using tf2
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()
        self.latest_img = None
        self.first_img = None
        self.first_cv_image = None

        self.pos_cam_in_bird_view_pxls = None
        self.bird_view = None
        self.map1 = None
        self.map2 = None

        self.platform_state = []


        self.subscription_boardCamera = self.create_subscription(
            Image,
            'board_camera/image_raw',
            self.board_camera_callback,
            10)
        
        self.board_camera_info = None

        # TODO: get camera info from board_camera/camera_info
        self.subscription_boardCam_info = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.board_camera_info_callback,
            10)
        
    def timer_callback(self):
        if self.latest_img is None:
            return
        
        if self.first_img is None:
            return
        # get camera info if not already
        if self.board_camera_info is None:
            self.get_logger().info("waiting for camera info...", once=True)
            return
        self.get_logger().info("camera info received", once=True)

        # ==================================== PREPROCESSING ====================================


        if self.bird_view is None:
            self.init_bird_view()

        if self.map1 is None:
            return

        # convert to cv2

        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img, "bgr8")

        #cv_image = cv2.imread("/home/sebastien/Board_Camera.png")

        if self.first_cv_image is None:
            self.first_cv_image = cv_image
            return
    
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # undistort
        cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        self.curent_image = cv_image

        # =================================== COMPUTE BIRD VIEW ===================================

        # get bird view
        bird_view_img = self.bird_view.project_img_to_bird(self.curent_image)

        cv2.imshow("Board Camera", bird_view_img)
        cv2.waitKey(1)

    def board_camera_callback(self, msg):
        if self.first_img is None:
            self.first_img = msg
        self.latest_img = msg

    def board_camera_info_callback(self, msg):
        self.board_camera_info = msg

    def init_bird_view(self):
        transform = None
        try:
            # get transform, which is what we need to wait for
            transform = self.tf_buffer.lookup_transform('base_link', 'camera',rclpy.time.Time().to_msg()) # todo use camera info
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
            K = np.array(self.board_camera_info.k).reshape(3,3)
            self.bird_view = bv.BirdView(K, transform_mtx, (0.16, -0.55), (0.65, 0.55), resolution=700)

            self.pos_cam_in_bird_view_pxls = self.bird_view.get_work_plane_pt_in_bird_img(np.array([0, 0, 1]))
            ic(self.pos_cam_in_bird_view_pxls)

            # compute undistortion map
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, np.array(self.board_camera_info.d), None, K, (self.board_camera_info.width,self.board_camera_info.height), cv2.CV_32FC1)

            self.get_logger().info("Node Initialized", once=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("waiting for tf2 transform...", once=True)
            return

def main(args=None):
    rclpy.init(args=args)
    board_plateforme_detection_node = BoardPlateformeDetectionNode()
    rclpy.spin(board_plateforme_detection_node)

    board_plateforme_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
