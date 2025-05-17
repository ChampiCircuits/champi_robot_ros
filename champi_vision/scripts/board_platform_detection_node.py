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

class PlatformePose:
    def __init__(self, pos1, pos2, theta):
        self.pos1 = pos1
        self.pos2 = pos2
        self.theta = theta

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

        # if we want to segment by segmentation color or keypoints matching
        self.segmentationByColor = False


        # Keypoint matching------------------------------------------------------

        # if we have the coordinates of the platforms for keypoints matching
        self.byCoordinates = True

        # path to the platform image
        self.platform_dir = "/home/sebastien/Pictures/Screenshots/plateform2.png"

        # descriptor to use for keypoints matching
        self.descriptor = 'SIFT'

        # crosscheck for keypoints matching (if True, it will only return the keypoints that are mutual) = 
        # descriptor A in image 1 matches descriptor B in image 2, and descriptor B matches descriptor A
        # set to true for more accuracy and less matchs
        self.crosscheck = False

        self.img_plateform = cv2.imread(self.platform_dir)
        
        # min number of keypoints to detect a platform
        self.numberMinKeypoints = 7
        # coordinates of the platforms in the bird view
        self.plateform_coordinates = [PlatformePose((190, 4), (286, 66), 0),
                                        PlatformePose((471, 4), (569, 69), 0),
                                        PlatformePose((50, 6), (133, 106), 270),
                                        PlatformePose((627, 12), (700, 110), 90),
                                        PlatformePose((236, 114), (337, 173), 0),
                                        PlatformePose((416, 117), (520, 172), 0),
                                        PlatformePose((0, 154), (83, 265), 270),
                                        PlatformePose((664, 165), (767, 276), 90),
                                        PlatformePose((126, 280), (262, 327), 0),
                                        PlatformePose((484, 287), (623, 334), 0)]
        
        # showing where the platform should be in the image in 16 parts of the image
        # (only on segmentation by keypoint and no by coordinates predifined)
        self.plateforme_grid_plateform = [[True, True, True, True],
                                        [False, True, True, False],
                                        [True, False, False, True],
                                        [True, False, True, False]]
        
        self.platform_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # DEBUG
        self.show_keypoint = False
        self.show_ROI = False


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

        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # undistortion
        cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        self.curent_image = cv_image

        # =================================== COMPUTE BIRD VIEW ===================================

        # get bird view
        bird_view_img = self.bird_view.project_img_to_bird(self.curent_image)

        # =================================== SEGMENTATION ===================================
        # TODO : enhance segmentation by color by detectiing rectangles and area of the platform
        if self.segmentationByColor:
        # get the first segmented image
            if self.first_cv_image is None:
                self.first_cv_image = self.segmentation_color(bird_view_img)
                return
            # get the segmented image
            segmented_img = self.segmentation_color(bird_view_img)

        # =================================== KEYPOINTS MATCHING ===================================
        else:
            if self.byCoordinates: # if we have the coordinates of the platforms
                for plateform in self.plateform_coordinates:
                    # get the segmented image
                    img_roi = bird_view_img[plateform.pos1[1]:plateform.pos2[1], plateform.pos1[0]:plateform.pos2[0]]
                    nbrKeypoints = self.segmentation_keypoints(img_roi)
                    if nbrKeypoints > self.numberMinKeypoints:
                        self.platform_state[self.plateform_coordinates.index(plateform)] = 1
                    else:
                        self.platform_state[self.plateform_coordinates.index(plateform)] = 0

                    if self.show_ROI:
                        plt.imshow(img_roi), plt.show()

            else: # if we don't have the coordinates of the platforms, divise the image in 16 parts
                height, width = bird_view_img.shape[:2]
                count = 0
                for i in range(4): # divise the image in 16 parts
                    for j in range(4):
                        if not self.plateforme_grid_plateform[i][j]: #if there is no platform in this part of the image
                            continue
                        if i == 3: # if it's the last row of the image (platforms are on 2 parts), take 2 parts for ROI
                            img_roi = bird_view_img[i*int(height/4):(i+1)*int(height/4), j*int(width/4):(j+2)*int(width/4)]
                        else:
                            img_roi = bird_view_img[i*int(height/4):(i+1)*int(height/4), j*int(width/4):(j+1)*int(width/4)]

                        nbrKeypoints = self.segmentation_keypoints(img_roi)
                        #print(nbrKeypoints)
                        if nbrKeypoints > self.numberMinKeypoints:
                            self.platform_state[count] = 1
                        else:
                            self.platform_state[count] = 0
                        
                        count += 1
                        if self.show_ROI:
                            plt.imshow(img_roi), plt.show()

        print(self.platform_state)

        cv2.imshow("Board Camera", bird_view_img)
        cv2.waitKey(1)

    def board_camera_callback(self, msg):
        if self.first_img is None:
            self.first_img = msg
        self.latest_img = msg

    def board_camera_info_callback(self, msg):
        self.board_camera_info = msg

    def segmentation_color(self, img):
        # Convert the image to the HSV color space
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h_img, s_img, v_img = cv2.split(img_hsv)

        # self.display_histogram(img_hsv)

        # Define the lower and upper bounds for the color segmentation
        lower_bound = np.array([13, 0, 0])
        upper_bound = np.array([15, 255, 255])

        # Create a mask for the color segmentation
        mask = cv2.inRange(img_hsv, lower_bound, upper_bound)

        # Apply the mask to the original image
        img_segmented = cv2.bitwise_and(img, img, mask=mask)

        img_segmented_gray = cv2.cvtColor(img_segmented, cv2.COLOR_BGR2GRAY)

        _, thresh_img_segmented_gray= cv2.threshold(img_segmented_gray, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #opening
        kernel = np.ones((5,5),np.uint8)
        thresh_img_segmented_gray = cv2.morphologyEx(thresh_img_segmented_gray, cv2.MORPH_OPEN, kernel)
        # cv2.imshow("Board Camera", thresh_img_segmented_gray)
        # cv2.waitKey(1)
        # TODO : find rectangle in the image
        return thresh_img_segmented_gray
    
    # detection platform by keypoints matching with SIFT (return the number of keypoints)
    def segmentation_keypoints(self, img):
        # Convert the image to the HSV color space
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_plateform_hsv = cv2.cvtColor(self.img_plateform, cv2.COLOR_BGR2HSV)
        # BRISK Detector
        detector = cv2.BRISK_create()

        # norme type (distance) for matching keypoint. More information here : https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
        norm_Type = cv2.NORM_L2

        # threshold for the distance between the keypoints
        threshold = 1000

        if self.descriptor == 'BRISK':   # very good for small images, but not many keypoints, fast and good for scale changes. Use Hamming Distance. 
            detector = cv2.BRISK_create()
            norm_Type = cv2.NORM_HAMMING
            threshold = 140
        elif self.descriptor == 'SIFT':    # more keypoints and very accurate and slower. Use Euclidean Distance
            detector = cv2.SIFT_create()
            norm_Type = cv2.NORM_L2
            threshold = 360
        elif self.descriptor == 'KAZE':  # less keypoints but less accurate than SIFT. use Euclidean Distance
            detector = cv2.KAZE_create()
            norm_Type = cv2.NORM_L2
            threshold = 0.5
        else:
            print("Descriptor not found")
            return 0
        
        # # Detect keypoints and descriptors
        kp1, des1 = detector.detectAndCompute(img_plateform_hsv, None)
        kp2, des2 = detector.detectAndCompute(img_hsv, None)

        if des1 is None:
            #print("Descriptors are None1. Ensure the images have enough features.")
            return 0
        if des2 is None:
            #print("Descriptors are None2. Ensure the images have enough features.")
            return 0

        # create BFMatcher object. Put crossCheck = True for better results.
        # normType : Norm_L2 for SIFT, SURF, BRIEF, BRISK, NORM_HAMMING for ORB, FREAK
        bf = cv2.BFMatcher(crossCheck=self.crosscheck, normType=norm_Type)

        # Match descriptors.
        matches = bf.match(des1,des2)

        # sort the matches based on distance
        matches = sorted(matches, key=lambda val: val.distance)

        # Apply a distance threshold
        good_matches = []
        for m in matches:
            if self.show_keypoint:
                print(m.distance)
            if m.distance < threshold:
                good_matches.append(m)

        if self.show_keypoint:
            # Draw first 50 matches.
            print('Number of keypoints in the img_plateform: ', len(kp1))
            print('Number of keypoints in the img: ', len(kp2))
            print('Number of good matches: ', len(good_matches))
            out = cv2.drawMatches(img_plateform_hsv, kp1, img_hsv, kp2, good_matches, None, flags=2)
            plt.imshow(out), plt.show()

        return len(good_matches)

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
