#!/usr/bin/env python3

# -*- coding: utf-8 -*-

"""ROS2 Camera Image Publish Executer.

This script publishes camera image to a ROS2 topic in sensor_msgs.msg/Image 
format. 

Revision History:
        2021-03-25 (Animesh): Baseline Software.

Example:
        $ colcon build && . install/setup.bash && ros2 run ros2_camera_publish execute

"""


#___Import Modules:
import os
import cv2
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory


#___Global Variables:
DEVICE_INDEX = 0 # specifies which camera
TOPIC = 'racecar/camera'
QUEUE_SIZE = 1
PERIOD = 0.1  # seconds


#__Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.

    This class contains all methods to publish camera data and info in ROS 2 
    topic in sensor_msgs.msg/Image format. 
    
    """

    
    def __init__(self, capture, topic=TOPIC, queue=QUEUE_SIZE, period=PERIOD):
        """Constructor.

        Args:
            capture: OpenCV Videocapture object.

        """

        super().__init__('camera_publisher')
        
        # initialize publisher
        self.publisher_ = self.create_publisher(Image, topic, queue)
        timer_period = period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # set image counter and videocapture object
        self.capture = capture
        self.i = 0

        if self.capture.isOpened():
            print("Camera opened")
        else:
            print("Could not open camera")

        
    def timer_callback(self):
        """Timer Callback Function
        
        This method captures images and publishes required data in ros 2 topic.
        
        """
            
        # reads image data
        ret, frame = self.capture.read()


        frame = cv2.resize(frame, (0,0), fx = 0.3, fy = 0.3)

        # processes image data and converts to ros 2 message
        msg = Image()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'ANI717'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(frame)[2] * np.shape(frame)[1]
        msg.data = np.array(frame).tobytes()

        print("width", msg.width)
        print('height', msg.height)

        # publishes message
        self.publisher_.publish(msg)
        
        # image counter increment
        self.i += 1
        
        return None


#___Main Method:
def main(args=None):
    """This is the Main Method.
    
    """
        
    # creates OpenCV Videocapture object
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    
    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(capture, "/camera", 10, 1.)
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()
    
    return None


#___Driver Program:
if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""