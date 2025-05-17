#!/usr/bin/env python3
import rclpy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2

from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge

from icecream import ic
from scipy.spatial.transform import Rotation as R
import champi_vision.bird_view as bv

class BirdViewPublisherNode(Node):
    def __init__(self):

        super().__init__('bird_view_publisher_node')

        # ==================================== BIRD VIEW ====================================
        # transformation matrix between base_link and camera using tf2


        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()
        self.latest_img = None

        self.pos_cam_in_bird_view_pxls = None
        self.bird_view = None
        self.map1 = None
        self.map2 = None


        # handle image
        self.imageRGB_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        # handle camera info
        self.champi_camera_info = None

        # subscribe to camera info
        self.subscription_boardCam_info = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.champi_camera_info_callback,
            10)
        
        self.bird_view_RGB_publisher = self.create_publisher(Image, '/champi_bird_view_RGB', 10)
        self.timer_bird_view_RGB = self.create_timer(0.1, self.publish_bird_view_RGB)  # 5Hz
        


    def publish_bird_view_RGB(self):
        if self.latest_img is None:
            return

        # get camera info if not already
        if self.champi_camera_info is None:
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

        #Blur the image
        #img_blured = cv2.GaussianBlur(self.cv_image, (7,7), 0)
        # img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # undistortion
        cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        self.curent_image = cv_image
        

        # =================================== COMPUTE BIRD VIEW ===================================

        # get bird view
        bird_view_img = self.bird_view.project_img_to_bird(self.curent_image)

        # =================================== PUBLISH BIRD VIEW ===================================
        msg = self.cv_bridge.cv2_to_imgmsg(bird_view_img, encoding="bgr8")
        self.bird_view_RGB_publisher.publish(msg)

        
    def init_bird_view(self):
        transform = None
        try:
            # get transform, which is what we need to wait for
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link',rclpy.time.Time().to_msg()) # todo use camera info
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
            K = np.array(self.champi_camera_info.k).reshape(3,3)
            self.bird_view = bv.BirdView(K, transform_mtx, (0.27, -0.4), (0.82, 0.4), resolution=378)

            self.pos_cam_in_bird_view_pxls = self.bird_view.get_work_plane_pt_in_bird_img(np.array([0, 0, 1]))
            ic(self.pos_cam_in_bird_view_pxls)

            # compute undistortion map
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, np.array(self.champi_camera_info.d), None, K, (self.champi_camera_info.width,self.champi_camera_info.height), cv2.CV_32FC1)

            self.get_logger().info("Node Initialized", once=True)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("waiting for tf2 transform...", once=True)
            return


    def image_callback(self, msg):
        self.latest_img = msg


    def champi_camera_info_callback(self, msg):
        self.champi_camera_info = msg


def main(args=None):
    rclpy.init(args=args)

    birdview_publisher = BirdViewPublisherNode()

    rclpy.spin(birdview_publisher)

    birdview_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()