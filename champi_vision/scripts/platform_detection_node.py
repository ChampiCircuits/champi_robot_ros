#!/usr/bin/env python3
import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32

from std_msgs.msg import Bool
from std_msgs.msg import Header
#from skimage.filters import threshold_sauvola

import cv2
import open3d as o3d
import itertools
import sys
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

from icecream import ic
from scipy.spatial.transform import Rotation as R
import champi_vision.bird_view as bv



class PlatformDetection(Node):
    def __init__(self):
        super().__init__('platform_detection')

        self.timer = self.create_timer(0.5, self.timer_callback)  # 5Hz

        self.cv_bridge = CvBridge()

        #self.publisher_platform = self.create_publisher(Bool, 'platform_detection', 10)

        self.subscription_camera = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.champi_camera_info = None

        # TODO: code plateforme state from the plateforme detected and the position of the robot
        self.plateforme_state = [False, False, False, False, False, False, False, False, False, False]

        self.platform_detected = False

        self.get_logger().warn("tests.")

        # ==================================== POINTCLOUD ====================================
        
        self.subscription_champi_pointcloud = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.champi_pointcloud_callback,
            10)
        
        self.transform = None
        self.plateformHeight = 0.10
        self.interval_plateformHeight = [0.01, 0.03]

        self.champi_pointcloud = None
        self.isPointCloudPublished = False
        self.isPlaneDetected = False
        
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/champi_pointcloud_filtered', 10)
        if self.isPointCloudPublished:
            self.timer_pointcloud = self.create_timer(0.1, self.publish_pointcloud)  # 5Hz
        # ==================================== BIRD VIEW ====================================

        # transformation matrix between base_link and camera using tf2
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cv_bridge = CvBridge()
        self.latest_img = None

        # NOT USED ==================================== COLOR ====================================
        # self.is_color_segmentation = True
        # self.numberMinPixel = 6000  # min number of pixel to detect a platform
        # self.edge_detector = "Canny"
    


        # NOT USED ==================================== KEYPOINTS ====================================
                # path to the platform image
        # self.platform_dir = "/home/sebastien/ws_champi/src/champi_robot_ros/champi_vision/ressources/images/plateform_champi2.png"

        # # descriptor to use for keypoints matching
        # self.descriptor = 'SIFT'

        # # crosscheck for keypoints matching (if True, it will only return the keypoints that are mutual) = 
        # # descriptor A in image 1 matches descriptor B in image 2, and descriptor B matches descriptor A
        # # set to true for more accuracy and less matchs
        # self.crosscheck = True

        # if not self.is_color_segmentation:
        #     self.img_plateform = cv2.imread(self.platform_dir)
        
        # # min number of keypoints to detect a platform
        # self.numberMinKeypoints = 7

        # ==================================== ARUCO ====================================
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters =  cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.miniArucoToDetect = 2

        # NOT USED ==================================== CALLIBRATION ====================================
        
        # In calibration mode, the color segmentation is activated
        # self.is_calibration = False
        # self.threshold_floodfill = 4

        # # Define the lower and upper bounds for the color segmentation
        # self.lower_bound = np.array([13, 0, 0])
        # self.upper_bound = np.array([15, 255, 255])

        # ==================================== DEBUG ====================================
        self.show_keypoint = True
        self.show_histogram = False
        self.show_edges = True
        self.show_mask = False


    def timer_callback(self):
        if self.latest_img is None:
            return

        if self.transform is None:
            self.init_transform()
            
               # convert to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')
        new_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        self.platform_detected = self.detectArucoOnPillars(new_image) and self.isPlaneDetected
        return
    
        # ==================================== PREPROCESSING WITH BIRD VIEW ====================================

        # # convert to cv2
        # bird_view_img = self.cv_bridge.imgmsg_to_cv2(self.latest_img, "bgr8")

        # #Blur the image
        # #img_blured = cv2.GaussianBlur(self.cv_image, (7,7), 0)
        # # img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # self.curent_image = bird_view_img


        # if self.is_calibration and self.is_color_segmentation:
        #     print("----------Calibration mode Plateforme detection----------")
        #     print("put the robot in front of a plateforme and save the Hue Saturation range of the plateforme")
        #     print("Change the Hue Saturation Value in the code or in the calibration.yaml file in the self.lower_bound and self.upper_bound")    
        #     img_hsv = cv2.cvtColor(bird_view_img, cv2.COLOR_BGR2HSV)
        #     plt.imshow(img_hsv)
        #     plt.show()
        #     sys.exit()
        #     return

        # #print('Image received')

        # # ================================== DETECTION =========================================
        # # cv2.imshow('Image', bird_view_img)
        # # cv2.waitKey(1)


        # img_edges = None

        # if self.is_color_segmentation:
        #     # Perform color segmentation
        #     img_edges = self.segmentation_color(bird_view_img)

        #     # Detect edges
        #     #img_edges = self.detect_edges(img_gray, self.edge_detector)

        #     # Detect lines
        #     #img_lines = detect_lines(img, img_edges)

        #     # Detect blobs
        #     #img_blobs = detect_blobs(img, img_edges)

        #      # find the countours
        #     contours, _ = cv2.findContours(img_edges, cv2.RETR_CCOMP , cv2.CHAIN_APPROX_SIMPLE)

        #     # Draw the rectangles on the original image
        #     img_contours = bird_view_img.copy()

        #     self.platform_detected = False
        #     for cnt in contours:

        #         area = cv2.contourArea(cnt)
        #         print("Area : ", area)
        #         # Filter contours based on size and/or aspect ratio
        #         # Adjust the conditions to match the shapes
        #         if area > self.numberMinPixel:

        #             # Detect rectangles
        #             rectangle = self.detect_rectangle(cnt)
        #             #cv2.drawContours(img_contours, [cnt], -1, (0, 255, 0), 2)  # Draw one shape in green
        #             if rectangle is not None:
        #                 cv2.drawContours(img_contours, rectangle, -1, (0, 255, 0), thickness=cv2.FILLED)
        #                 print("Rectangles found")
        #                 self.platform_detected = True

        #     if self.show_edges:
        #         cv2.imshow('Edges', img_contours)
        #         cv2.waitKey(1)


        # else:
        #     nbrKeypoints = self.segmentation_keypoints(bird_view_img)
        #     #print(nbrKeypoints)
        #     if nbrKeypoints > self.numberMinKeypoints:
        #         self.platform_detected = True
        #         print("Plateforme detected")
        #     else:
        #         self.platform_detected = False

        # msg = Bool()
        # msg.data = self.platform_detected
        # #self.publisher_platform.publish(msg)
        # # cv2.imshow('Image', img_contours)
        # # cv2.waitKey(1)
    def detectArucoOnPillars(self, img):
        # Equalize the histogram 5enhqnce contrast
        new_image = cv2.equalizeHist(img)

        #apply adaptative threshold
        new_image = cv2.adaptiveThreshold(new_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY, 11, 2) #best
        
        # thresh_value = thresh_value = threshold_sauvola(new_image, window_size=15)
        # new_image = (new_image > thresh_value).astype('uint8') * 255 #best

        # cv2.imshow("test", new_image)
        # cv2.waitKey(1)
        markerCorners, _, _ = self.aruco_detector.detectMarkers(new_image)

        print("nrb markers:", len(markerCorners))
        if len(markerCorners) >= self.miniArucoToDetect:
            return True
        else:
            return False
        
    def image_callback(self, msg):
        self.latest_img = msg

    def champi_pointcloud_callback(self, msg):

        if self.transform is None:
            return

        # Read structured point cloud data
        structured_points = np.array(list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        if structured_points.size == 0:
            self.get_logger().warn("Received empty point cloud.")
            return

        # ====================================TRANSFORM POINT CLOUD TO BASE LINK ====================================
        # Convert to numpy array
        points = np.vstack([structured_points['x'], structured_points['y'], structured_points['z']]).T

        # Ensure it's float32
        points = points.astype(np.float32)

        # Step 1: Convert to homogeneous coordinates (N,3) → (N,4)
        ones = np.ones((points.shape[0], 1))  # Shape: (N,1)

        points_homogeneous = np.concatenate([points, ones], axis=1)  

        # Step 2: Apply the transformation
        transformed_points_homogeneous = points_homogeneous @ self.transform.T  # Matrix multiplication

        # Step 3: Convert back to (N,3) by removing the homogeneous coordinate
        transformed_points = transformed_points_homogeneous[:, :3]
       # new_points = points @ self.transform
        #points = np.array(list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        # ==================================== FILTER POINT CLOUD ====================================
        self.champi_pointcloud = []
        for point in transformed_points:
            x, y, z = point
            if (self.plateformHeight - self.interval_plateformHeight[0]) < z < (self.plateformHeight + self.interval_plateformHeight[1]):
                self.champi_pointcloud.append(point)

        #self.champi_pointcloud = transformed_points

        # Detect planes
        self.isPlaneDetected = self.detect_plane(self.champi_pointcloud)

    # ==================================== INIT TRANSFORM FUNCTION POINT CLOUD FROM BASE LINK TO CAMERA ====================================

    def init_transform(self):
        try:
            
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link',rclpy.time.Time().to_msg())

            #get the rotation and translation matrix
            rot = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            rot = rot.as_matrix()
            trans = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

            #print(rot)
            ic(trans)

            # create the transformation matrix
            transform_mtx = np.concatenate([rot, trans.reshape(3,1)], axis=1)
            transform_mtx = np.concatenate([transform_mtx, np.array([[0,0,0,1]])], axis=0)

            self.transform = transform_mtx

            self.get_logger().warn("Transform initialized.")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Transform not initialized.")
            return

    # ==================================== PLANES DETECTION POINTCLOUD ====================================
    def segment_planes(self, pcd, distance_threshold=0.02, ransac_n=3, num_iterations=1000, min_points=100):
        """
        Segment multiple planes from the point cloud.
        Returns a list of tuples: (plane_model, inlier_indices)
        """
        planes = []
        rest = pcd
        while True:

            #Check if the remaining point cloud ('rest') has fewer points than the minimum required.
            if np.asarray(rest.points).shape[0] < min_points:
                break

            # Detect the plane
            # 5. Segment a plane from the current remaining point cloud using RANSAC.
        #    - distance_threshold: Maximum distance a point can be from the plane model to be considered an inlier.
        #    - ransac_n: Number of points to sample for generating a plane candidate.
        #    - num_iterations: How many iterations the RANSAC algorithm should run.
        #    The function returns:
        #       * plane_model: A list or array [a, b, c, d] representing the plane equation (ax + by + cz + d = 0).
        #       * inliers: A list of indices of the points that fit the plane model within the given threshold.
            plane_model, inliers = rest.segment_plane(distance_threshold=distance_threshold,
                                                        ransac_n=ransac_n,
                                                        num_iterations=num_iterations)
            
            # Check if the detected plane has enough inliers
            if len(inliers) < min_points:
                break
            planes.append(plane_model)
            # Remove the detected plane from the cloud
            rest = rest.select_by_index(inliers, invert=True)
        return planes


    def detect_plane(self, pointcloud):

        # 1. Load the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)

        # 2. Preprocess: Downsample for speed (adjust voxel_size as needed)  
        # downsampled point cloud where each 1 cm³ region is represented by one point to reduce computation
        pcd = pcd.voxel_down_sample(voxel_size=0.0025)

        # (Optional) Remove statistical outliers:
        #pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # 3. Segment planes using RANSAC
        planes = self.segment_planes(pcd,
                                distance_threshold=0.02,
                                ransac_n=3,
                                num_iterations=1000,
                                min_points=2000)
        print(f"Detected {len(planes)} planes.")
        if len(planes) == 0:
            return False
        else:
            return True


    def publish_pointcloud(self):
        if self.champi_pointcloud is None:
            return
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        point_cloud = create_cloud_xyz32(msg.header, self.champi_pointcloud)
        self.point_cloud_publisher.publish(point_cloud)
        #self.get_logger().warn("published.")


    # NOT USED ==================================== SEGMENTATION ====================================
    # def segmentation_keypoints(self, img):
    #     # Convert the image to the HSV color space
    #     img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     img_plateform_hsv = cv2.cvtColor(self.img_plateform, cv2.COLOR_BGR2HSV)
    #     # BRISK Detector
    #     detector = cv2.BRISK_create()

    #     # norme type (distance) for matching keypoint. More information here : https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
    #     norm_Type = cv2.NORM_L2

    #     # threshold for the distance between the keypoints
    #     threshold = 1000

    #     if self.descriptor == 'BRISK':   # very good for small images, but not many keypoints, fast and good for scale changes. Use Hamming Distance. 
    #         detector = cv2.BRISK_create()
    #         norm_Type = cv2.NORM_HAMMING
    #         threshold = 140
    #     elif self.descriptor == 'SIFT':    # more keypoints and very accurate and slower. Use Euclidean Distance
    #         detector = cv2.SIFT_create()
    #         norm_Type = cv2.NORM_L2
    #         threshold = 300
    #     elif self.descriptor == 'KAZE':  # less keypoints but less accurate than SIFT. use Euclidean Distance
    #         detector = cv2.KAZE_create()
    #         norm_Type = cv2.NORM_L2
    #         threshold = 0.5
    #     else:
    #         print("Descriptor not found")
    #         return 0
        
    #     # # Detect keypoints and descriptors
    #     kp1, des1 = detector.detectAndCompute(img_plateform_hsv, None)
    #     kp2, des2 = detector.detectAndCompute(img_hsv, None)

    #     if des1 is None:
    #         #print("Descriptors are None1. Ensure the images have enough features.")
    #         return 0
    #     if des2 is None:
    #         #print("Descriptors are None2. Ensure the images have enough features.")
    #         return 0

    #     # create BFMatcher object. Put crossCheck = True for better results.
    #     # normType : Norm_L2 for SIFT, SURF, BRIEF, BRISK, NORM_HAMMING for ORB, FREAK
    #     bf = cv2.BFMatcher(crossCheck=self.crosscheck, normType=norm_Type)

    #     # Match descriptors.
    #     matches = bf.match(des1,des2)

    #     # sort the matches based on distance
    #     matches = sorted(matches, key=lambda val: val.distance)

    #     # Apply a distance threshold
    #     good_matches = []
    #     for m in matches:
    #         if self.show_keypoint:
    #             print(m.distance)
    #         if m.distance < threshold:
    #             good_matches.append(m)

    #     if self.show_keypoint:
    #         # Draw first 50 matches.
    #         print('Number of keypoints in the img_plateform: ', len(kp1))
    #         print('Number of keypoints in the img: ', len(kp2))
    #         print('Number of good matches: ', len(good_matches))
    #         out = cv2.drawMatches(img_plateform_hsv, kp1, img_hsv, kp2, good_matches, None, flags=2)
    #         plt.imshow(out), plt.show()

    #     return len(good_matches)

    # def display_histogram(self, img):
    #        # Vérifier si l'image est en couleur ou en niveaux de gris
    #     max_val = np.array([0, 0, 0])
    #     if len(img.shape) == 2:  # Image en niveaux de gris
    #         hist = cv2.calcHist([img], [0], None, [256], [1, 256])
    #         max_val[0] = np.argmax(hist)
    #         plt.plot(hist, color='gray')
    #         plt.title("Histogramme (Niveaux de gris)m max : " + str(max_val))
    #         plt.xlabel("Valeurs de pixels")
    #         plt.ylabel("Fréquence")
    #         plt.show()
    #     else:  # Image en couleur (BGR)
    #         colors = ('b', 'g', 'r')  # Canaux B, G, R
    #         plt.figure()
    #         plt.title("Histogramme (Couleur)")
    #         plt.xlabel("Valeurs de pixels")
    #         plt.ylabel("Fréquence")

    #         for i, color in enumerate(colors):
    #             hist = cv2.calcHist([img], [i], None, [256], [1, 256])
    #             max_val[i] = np.argmax(hist)
    #             plt.plot(hist, color=color)
    #         plt.show()
    #     print("Max values : ", max_val)
    #     plt.imshow(img)
    #     plt.show()


    # # Segmentqtion by color 
    # def segmentation_color(self, img):
    #     # Convert the image to the HSV color space
    #     img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     h_img, s_img, v_img = cv2.split(img_hsv)

    #     #self.display_histogram(img_hsv)

    #     # Create a mask for the color segmentation
    #     mask = cv2.inRange(img_hsv, self.lower_bound, self.upper_bound)

    #     # Apply the mask to the original image
    #     img_segmented = cv2.bitwise_and(img, img, mask=mask)

    #     img_segmented_gray = cv2.cvtColor(img_segmented, cv2.COLOR_BGR2GRAY)

    #     _, thresh_img_segmented_gray= cv2.threshold(img_segmented_gray, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #     #apply morphological operations to the image
    #     opening = cv2.morphologyEx(thresh_img_segmented_gray, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))

    #     if self.show_mask:
    #         cv2.imshow('Mask', opening)
    #         cv2.waitKey(1)

    #     return opening

   

    # NOT USED ==================================== LINES ====================================

    # Function to calculate slope
    # def calculate_slope(self, x1, y1, x2, y2):
    #     if x2 - x1 == 0:  # To avoid division by zero for vertical lines
    #         return float('inf')  # Infinite slope for vertical lines
    #     return (y2 - y1) / (x2 - x1)

    # def is_point_on_line(self, x1, y1, x2, y2, x, y):
    #     if x1 == x2:
    #         return abs(x - x1) < 0.5
    #     if y1 == y2:
    #         return (y == y1) < 0.5
    #     return abs((x - x1) / (x2 - x1) - (y - y1) / (y2 - y1)) < 0.5

    # def is_parallel_lines(self, x1, y1, x2, y2, x3, y3, x4, y4):
    #     slope_line1 = self.calculate_slope(x1, y1, x2, y2)
    #     slope_line2 = self.calculate_slope(x3, y3, x4, y4)
    #     print("Parallel lines")
    #     return abs(slope_line1-slope_line2) < 2

    # def detect_lines(self, img, edges):

    #     # Use Hough Line Transform to detect lines
    #     lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80, minLineLength=25, maxLineGap=15)

    #     # Draw the detected lines on the original image
    #     output = img.copy()
    #     parallel_lines = None
        
    #     min_norm = 999
    #     if lines is not None and len(lines) > 1:
    #         print("Nbr lines", len(lines))
    #         for i in range(len(lines)):
    #             for j in range(i+1, len(lines)):
    #                 x1, y1, x2, y2 = lines[i][0]
    #                 if y1 > y2:
    #                     x2, y2, x1, y1 = x1, y1, x2, y2
    #                 x3, y3, x4, y4 = lines[j][0]
    #                 if y3 > y4:
    #                     x4, y4, x3, y3 = x3, y3, x4, y4

    #                 # Check if the line are the same
    #                 if self.is_point_on_line(x1, y1, x2, y2, x3, y3):
    #                     print("Point on line")
    #                     continue

    #                 if not self.is_parallel_lines(x1, y1, x2, y2, x3, y3, x4, y4):
    #                     print("Not parallel lines")
    #                     continue
                    
    #                 norm_line1 = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
    #                 norm_line2 = np.linalg.norm(np.array([x3, y3]) - np.array([x4, y4]))

    #                 # Check if the lines are the closest parallel lines
    #                 norm = abs(norm_line1 - norm_line2)
    #                 if norm < min_norm:
    #                     parallel_lines = (lines[i][0], lines[j][0])
    #                     min_norm = norm
            
    #         if parallel_lines is not None:
    #             line1, line2 = parallel_lines
    #             x1, y1, x2, y2 = line1
    #             x3, y3, x4, y4 = line2
    #             output = cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #             output = cv2.line(output, (x3, y3), (x4, y4), (0, 255, 0), 2)
        

    #     elif lines is not None and len(lines) == 1:
    #         print("1 lines detected")
    #         x1, y1, x2, y2 = lines[0][0]
    #         output = cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2)

    #     else:
    #         print("No lines detected")
    #         output = edges

    #     return output

    # NOT USED ==================================== EDGES ====================================
    # def detect_edges(self, img_gray, edge_detector):

    #     transformed_image = None

    #     if edge_detector == "Laplace":

    #         # Use Laplace edge detection
    #         sharpened_laplace = cv2.Laplacian(img_gray, cv2.CV_16U, ksize=5)

    #         # Convert the image to 8-bit unsigned integer
    #         sharpened_laplace_abs = cv2.convertScaleAbs(sharpened_laplace)

    #         # Apply a binary threshold to the image
    #         _, thresh_sharpened_laplace_abs= cv2.threshold(sharpened_laplace_abs, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #         # Apply morphological operarions to the image
    #         closing_laplace = cv2.morphologyEx(thresh_sharpened_laplace_abs, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
    #         transformed_image = cv2.morphologyEx(closing_laplace, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))

    #     elif edge_detector == "Sobel":

    #         # Use Sobel edge detection
    #         sobelx = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
    #         sobely = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
    #         sobelxy = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection

    #         #assemble the edges
    #         sharpened_sobel = cv2.addWeighted(sobelx, 0.5, sobely, 0.5, 0)
    #         sharpened_sobel = cv2.addWeighted(sharpened_sobel, 0.5, sobelxy, 0.5, 0)
    #         sharpened_sobel_abs = cv2.convertScaleAbs(sharpened_sobel)

    #         _, thresh_sharpened_sobel_abs = cv2.threshold(sharpened_sobel_abs, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #         transformed_image = cv2.morphologyEx(thresh_sharpened_sobel_abs, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    #     # transformed_image = cv2.morphologyEx(closing_sobel, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))

    #     elif edge_detector == "Canny":
    #         transformed_image = cv2.Canny(img_gray, 50, 200, apertureSize=3)
    #         #transformed_image = cv2.morphologyEx(sharpened_canny, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))
        
    #     return transformed_image

    # NOT USED ==================================== RECTANGLES ====================================
    # def detect_rectangle(self, contour):
    #     # Approximate contour with accuracy proportional to contour perimeter
    #     epsilon = 0.02 * cv2.arcLength(contour, True)
    #     approx = cv2.approxPolyDP(contour, epsilon, True)

    #     # Check if the approximated contour has 4 points (indicating a quadrilateral)
    #     if len(approx) == 4:
    #         # Optionally, check for convexity to ensure it's a proper shape
    #         if cv2.isContourConvex(approx):
    #             return approx
    #     return None

    # NOT USED==================================== BLOBS ====================================

    # def detect_blobs(self, img, edges):

    #     # Setup SimpleBlobDetector parameters.
    #     params = cv2.SimpleBlobDetector_Params()
        
    #     # Change thresholds
    #     params.minThreshold = 10
    #     params.maxThreshold = 200
        
    #     # Filter by Area.
    #     params.filterByArea = True
    #     params.minArea = 1000
        
    #     # Filter by Circularity
    #     params.filterByCircularity = True
    #     params.minCircularity = 0.05
    #     params.maxCircularity = 0.9
        
    #     # Filter by Convexity
    #     params.filterByConvexity = True
    #     params.minConvexity = 0.1
        
    #     # Filter by Inertia
    #     params.filterByInertia = True
    #     params.minInertiaRatio = 0.01

    #     blob_detector = cv2.SimpleBlobDetector_create(params)

    #     # Detect blobs.
    #     keypoints_canny = blob_detector.detect(edges)

    #     # Draw detected blobs as red circles.
    #     img_with_keypoints_canny = cv2.drawKeypoints(img, keypoints_canny, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #     return img_with_keypoints_canny

        
def main(args=None):
    rclpy.init(args=args)
    print("init")
    platform_detection = PlatformDetection()

    rclpy.spin(platform_detection)

    platform_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
