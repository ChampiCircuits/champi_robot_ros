#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class PlatformDetection(Node):
    def __init__(self):
        super().__init__('platform_detection')

        self.timer = self.create_timer(0.1, self.timer_callback)  # 5Hz

        self.cv_bridge = CvBridge()

        self.publisher_platform = self.create_publisher(Bool, 'platform_detection', 10)

        self.subscription_camera = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.is_color_segmentation = True
        self.latest_img = None
        self.edge_detector = "Canny"
    
        self.platform_detected = False

        # ==================================== CALLIBRATION ====================================
        
        # In calibration mode, the color segmentation is activated
        self.is_calibration = False
        self.threshold_floodfill = 4

        # Define the lower and upper bounds for the color segmentation
        self.lower_bound = np.array([8, 140, 0])
        self.upper_bound = np.array([15, 255, 255])



    def timer_callback(self):
        if self.latest_img is None:
            return

        #print('Image received')

        # ==================================== PREPROCESSING ====================================
        
        # Convert the ROS image to OpenCV image
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img, 'bgr8')

        if self.is_calibration and self.is_color_segmentation:
            print("----------Calibration mode Plateforme detection----------")
            print("put the robot in front of a plateforme and save the Hue Saturation range of the plateforme")
            print("Change the Hue Saturation Value in the code or in the calibration.yaml file in the self.lower_bound and self.upper_bound")    
            img_hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            plt.imshow(img_hsv)
            plt.show()
            sys.exit()
            return
        #Blur the image
        # img_blured = cv2.GaussianBlur(self.cv_image, (7,7), 0)
        # img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #print('Image encoding: ', img_gray.shape)
        # ================================== DETECTION =========================================
        
        img_edges = None
        if self.is_color_segmentation:
            # Perform color segmentation
            img_edges = self.segmentation_color(self.cv_image)



        # Detect edges
        #img_edges = self.detect_edges(img_gray, self.edge_detector)

        # Detect lines
        #img_lines = detect_lines(img, img_edges)

        # Detect blobs
        #img_blobs = detect_blobs(img, img_edges)

        # find the countours
        contours, _ = cv2.findContours(img_edges, cv2.RETR_CCOMP , cv2.CHAIN_APPROX_SIMPLE )

        # Detect rectangles
        rectangles = []
        rectangles = self.detect_rectangles(contours)

        # Draw the rectangles on the original image
        img_contours = self.cv_image.copy()
        if len(rectangles) > 0:
            cv2.drawContours(img_contours, rectangles, -1, (0, 255, 0), thickness=cv2.FILLED)
            print("Rectangles found")
            self.platform_detected = True
        else:
            print("No rectangles found")
            self.platform_detected = False
            # Detect blobs
            #img_contours = detect_blobs(img, img_edges)
            # sharpened_canny = cv2.Canny(img_gray, 25, 175, apertureSize=3)
            # transformed_image = cv2.morphologyEx(sharpened_canny, cv2.MORPH_CLOSE, np.ones((11,11),np.uint8))
            # #dilation_img = cv2.dilate(transf ormed_image, np.ones((3,3),np.uint8), iterations=1)
            # new_contours, _ = cv2.findContours(transformed_image, cv2.RETR_CCOMP , cv2.CHAIN_APPROX_SIMPLE)
            # # rectangles = detect_rectangles(new_contours)
            # # cv2.drawContours(img_contours, new_contours, -1, 255, thickness=cv2.FILLED)

            # for cnt in new_contours:
            #     area = cv2.contourArea(cnt)
            #     x, y, w, h = cv2.boundingRect(cnt)
            #     aspect_ratio = float(w) / h
                
            #     # Filter contours based on size and/or aspect ratio
            #     # Adjust the conditions to match the shapes
            #     if area > 4000 and aspect_ratio > 0.5:
            #         cv2.drawContours(img_contours, [cnt], -1, (0, 255, 0), 2)  # Draw one shape in green

        msg = Bool()
        msg.data = self.platform_detected
        self.publisher_platform.publish(msg)
        cv2.imshow('Image', img_contours)
        cv2.waitKey(1)
        #Solution : essayer un nouvequ edge detector
        #Solution : essayer de changer les paramètres de detection des blobs

    def value_regionalGrowing(self, img):
        
        new_img = np.zeros_like(img)

        # add Padding
        padded_img = np.pad(img, pad_width=1, mode='constant', constant_values=0)

        #print(padded_img)
        # applying the regional growing algorithm
        for i, lines in enumerate(img):
            for j, pixel in enumerate(lines):
                if i == 0 or j == 0 or i == img.shape[0] - 1 or j == img.shape[1] - 1:
                    continue
                window = np.array(padded_img[i:i+3, j:j+3]).astype(np.int32)
                mask = (np.abs(window - pixel) < self.threshold_floodfill)
                mask[1, 1] = False
                indices = np.where(mask==True)
                # if i> 194 and j > 63:
                #     print("Indices", indices)
                #     print("Window", window)
                #     print("Pixel", pixel)
                #     print("Mask", mask)
                #     print(np.abs(window - pixel))
                #     plt.imshow(new_img[i-1:i+2, j-1:j+2])
                #     plt.show()
                for k in range(indices[0].size):
                    if new_img[i-1+indices[0][k], j-1+indices[1][k]] != 0:
                        new_img[i, j] = new_img[i-1+indices[0][k], j-1+indices[1][k]]
                        break
                #print(mask)
                if new_img[i, j] == 0:
                    new_img[i, j] = np.random.randint(1, 255)

              #  window[mask] = pixel
                #new_img[i-1:i+2, j-1:j+2] = window

                #print(window)
        #print(new_img)
        # Apply a binary threshold to the image
        # _, thresh_img_segmented_region_growing = cv2.threshold(img, 254, 255, cv2.THRESH_BINARY)

        # return thresh_img_segmented_region_growing

    def display_histogram(self, img):
           # Vérifier si l'image est en couleur ou en niveaux de gris
        max_val = np.array([0, 0, 0])
        if len(img.shape) == 2:  # Image en niveaux de gris
            hist = cv2.calcHist([img], [0], None, [256], [1, 256])
            max_val[0] = np.argmax(hist)
            plt.plot(hist, color='gray')
            plt.title("Histogramme (Niveaux de gris)m max : " + str(max_val))
            plt.xlabel("Valeurs de pixels")
            plt.ylabel("Fréquence")
            plt.show()
        else:  # Image en couleur (BGR)
            colors = ('b', 'g', 'r')  # Canaux B, G, R
            plt.figure()
            plt.title("Histogramme (Couleur)")
            plt.xlabel("Valeurs de pixels")
            plt.ylabel("Fréquence")

            for i, color in enumerate(colors):
                hist = cv2.calcHist([img], [i], None, [256], [1, 256])
                max_val[i] = np.argmax(hist)
                plt.plot(hist, color=color)
            plt.show()
        print("Max values : ", max_val)
        plt.imshow(img)
        plt.show()

    def segmentation_color(self, img):
        # Convert the image to the HSV color space
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h_img, s_img, v_img = cv2.split(img_hsv)

        self.display_histogram(img_hsv)

        # Define the lower and upper bounds for the color segmentation
        self.lower_bound = np.array([8, 140, 0])
        self.upper_bound = np.array([15, 255, 255])

        # Create a mask for the color segmentation
        mask = cv2.inRange(img_hsv, self.lower_bound, self.upper_bound)

        # Apply the mask to the original image
        img_segmented = cv2.bitwise_and(img, img, mask=mask)

        img_segmented_gray = cv2.cvtColor(img_segmented, cv2.COLOR_BGR2GRAY)

        _, thresh_img_segmented_gray= cv2.threshold(img_segmented_gray, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        return thresh_img_segmented_gray

    def image_callback(self, msg):

        self.latest_img = msg

        # Function to calculate slope
    def calculate_slope(self, x1, y1, x2, y2):
        if x2 - x1 == 0:  # To avoid division by zero for vertical lines
            return float('inf')  # Infinite slope for vertical lines
        return (y2 - y1) / (x2 - x1)

    def is_point_on_line(self, x1, y1, x2, y2, x, y):
        if x1 == x2:
            return abs(x - x1) < 0.5
        if y1 == y2:
            return (y == y1) < 0.5
        return abs((x - x1) / (x2 - x1) - (y - y1) / (y2 - y1)) < 0.5

    def is_parallel_lines(self, x1, y1, x2, y2, x3, y3, x4, y4):
        slope_line1 = self.calculate_slope(x1, y1, x2, y2)
        slope_line2 = self.calculate_slope(x3, y3, x4, y4)
        print("Parallel lines")
        return abs(slope_line1-slope_line2) < 2

    def detect_lines(self, img, edges):

        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80, minLineLength=25, maxLineGap=15)

        # Draw the detected lines on the original image
        output = img.copy()
        parallel_lines = None
        
        min_norm = 999
        if lines is not None and len(lines) > 1:
            print("Nbr lines", len(lines))
            for i in range(len(lines)):
                for j in range(i+1, len(lines)):
                    x1, y1, x2, y2 = lines[i][0]
                    if y1 > y2:
                        x2, y2, x1, y1 = x1, y1, x2, y2
                    x3, y3, x4, y4 = lines[j][0]
                    if y3 > y4:
                        x4, y4, x3, y3 = x3, y3, x4, y4

                    # Check if the line are the same
                    if self.is_point_on_line(x1, y1, x2, y2, x3, y3):
                        print("Point on line")
                        continue

                    if not self.is_parallel_lines(x1, y1, x2, y2, x3, y3, x4, y4):
                        print("Not parallel lines")
                        continue
                    
                    norm_line1 = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
                    norm_line2 = np.linalg.norm(np.array([x3, y3]) - np.array([x4, y4]))

                    # Check if the lines are the closest parallel lines
                    norm = abs(norm_line1 - norm_line2)
                    if norm < min_norm:
                        parallel_lines = (lines[i][0], lines[j][0])
                        min_norm = norm
            
            if parallel_lines is not None:
                line1, line2 = parallel_lines
                x1, y1, x2, y2 = line1
                x3, y3, x4, y4 = line2
                output = cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
                output = cv2.line(output, (x3, y3), (x4, y4), (0, 255, 0), 2)
        

        elif lines is not None and len(lines) == 1:
            print("1 lines detected")
            x1, y1, x2, y2 = lines[0][0]
            output = cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2)

        else:
            print("No lines detected")
            output = edges

        return output

    def detect_edges(self, img_gray, edge_detector):

        transformed_image = None

        if edge_detector == "Laplace":

            # Use Laplace edge detection
            sharpened_laplace = cv2.Laplacian(img_gray, cv2.CV_16U, ksize=5)

            # Convert the image to 8-bit unsigned integer
            sharpened_laplace_abs = cv2.convertScaleAbs(sharpened_laplace)

            # Apply a binary threshold to the image
            _, thresh_sharpened_laplace_abs= cv2.threshold(sharpened_laplace_abs, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            # Apply morphological operarions to the image
            closing_laplace = cv2.morphologyEx(thresh_sharpened_laplace_abs, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
            transformed_image = cv2.morphologyEx(closing_laplace, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))

        elif edge_detector == "Sobel":

            # Use Sobel edge detection
            sobelx = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
            sobely = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
            sobelxy = cv2.Sobel(src=img_gray, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection

            #assemble the edges
            sharpened_sobel = cv2.addWeighted(sobelx, 0.5, sobely, 0.5, 0)
            sharpened_sobel = cv2.addWeighted(sharpened_sobel, 0.5, sobelxy, 0.5, 0)
            sharpened_sobel_abs = cv2.convertScaleAbs(sharpened_sobel)

            _, thresh_sharpened_sobel_abs = cv2.threshold(sharpened_sobel_abs, 254, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            transformed_image = cv2.morphologyEx(thresh_sharpened_sobel_abs, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        # transformed_image = cv2.morphologyEx(closing_sobel, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))

        elif edge_detector == "Canny":
            sharpened_canny = cv2.Canny(img_gray, 50, 200, apertureSize=3)
            transformed_image = cv2.morphologyEx(sharpened_canny, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))
        
        return transformed_image

    def detect_rectangles(self, contours):
        rectangles = []
        for contour in contours:
            # Approximate contour with accuracy proportional to contour perimeter
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximated contour has 4 points (indicating a quadrilateral)
            if len(approx) == 4:
                # Optionally, check for convexity to ensure it's a proper shape
                if cv2.isContourConvex(approx):
                    rectangles.append(approx)
        return rectangles

    def detect_blobs(self, img, edges):

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1000
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.05
        params.maxCircularity = 0.9
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.1
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        blob_detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints_canny = blob_detector.detect(edges)

        # Draw detected blobs as red circles.
        img_with_keypoints_canny = cv2.drawKeypoints(img, keypoints_canny, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return img_with_keypoints_canny

        
def main(args=None):
    rclpy.init(args=args)

    platform_detection = PlatformDetection()

    rclpy.spin(platform_detection)

    platform_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
