#!/usr/bin/env python3

import numpy as np
from icecream import ic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from spatz_interfaces.msg import BirdEyeParam

import cv2


class RvizImageDisplayer:
    """Class to display an image in rviz using a Rviz plugin.
    """
    def __init__(self, node, pxl_per_meter, image_with_m, image_height_m, frame_id):
        """Constructor.

        Args:
            node (rclpy.node Node): Node to create the publishers and timers.
            pxl_per_meter (float): Resolution of the image.
            image_with_m (float): Dimension of the image in meters, along the x axis of the frame_id frame.
            image_height_m (float): Dimension of the image in meters, along the y axis of the frame_id frame.
            frame_id (string): Frame id of the origin of the image.
        """

        self.node = node

        self.img_size = np.array([image_with_m * pxl_per_meter, image_height_m * pxl_per_meter]).astype(int)

        ic(self.img_size)

        self.publish_period_img = 0.1
        self.publish_period_param = 0.5

        # This is a topic required by the rviz plugin to display the image. 
        # It must be published on the topic /params
        self.bird_eye_param = BirdEyeParam()
        self.bird_eye_param.header.frame_id = frame_id
        self.bird_eye_param.header.stamp = self.node.get_clock().now().to_msg()
        self.bird_eye_param.width = int(self.img_size[0])
        self.bird_eye_param.height = int(self.img_size[1])
        self.bird_eye_param.resolution = pxl_per_meter
        self.bird_eye_param.offset.x = 0. # TODO make it a parameter
        self.bird_eye_param.offset.y = 0.
        self.bird_eye_param.offset.z = 0.

        self.img = Image()
        self.img.header.frame_id = self.bird_eye_param.header.frame_id
        self.img.height = int(self.img_size[1])
        self.img.width = int(self.img_size[0])
        self.img.encoding = 'rgb8'
        self.img.is_bigendian = 0
        self.img.step = self.img.width * 3
        
        self.background_img = None
        self.background_img_bkp = None # Used to reset the background image
        
        self.image_pub = self.node.create_publisher(Image, 'bird_eye_image', 10)
        self.bird_eye_param_pub = self.node.create_publisher(BirdEyeParam, '/params', 10)

        # Timer for the /bird_eye_image topic
        # self.timer_img = self.node.create_timer(self.publish_period_img, self.publish_image)

        # Timer for the /params topic
        self.timer_param = self.node.create_timer(self.publish_period_param, self.publish_param)


    def publish_image(self):
        """Publish the image
        """
        if self.background_img is None:
            return
        
        self.img.header.stamp = self.node.get_clock().now().to_msg()
        self.img.data = self.background_img.tobytes()
        self.image_pub.publish(self.img)

    
    def publish_param(self):
        """Publish the BirdEyeParam message. This message is required by the rviz plugin to display the image.
        """

        if self.background_img is None:
            return
        
        self.bird_eye_param_pub.publish(self.bird_eye_param)
    
    def set_background_img(self, img):
        self.background_img = img.copy()
        self.background_img_bkp = img.copy()
    
    def reset_to_background(self):
        self.background_img = self.background_img_bkp.copy()
    
    def load_background_img(self, path):
        self.background_img = cv2.imread(path)
        self.background_img = cv2.cvtColor(self.background_img, cv2.COLOR_BGR2RGB)
        self.background_img = cv2.resize(self.background_img, (self.img_size[0], self.img_size[1]))

        self.background_img_bkp = self.background_img.copy()

    def m_to_pxl(self, m):
        m = np.array(m)
        tmp = m * self.bird_eye_param.resolution
        tmp[:, 1] = self.img_size[1] - tmp[:, 1]
        return tmp.astype(int)
    
    def get_img(self):
        """Method to access the image, so you can draw on it.

        Returns:
            np.array: The image (as reference)
        """
        return self.background_img
    
    def update(self):
        """ Call this method to update the image in rviz.
        It will reset the image to the background image afterwards.
        """
        self.publish_image()
        self.reset_to_background()


