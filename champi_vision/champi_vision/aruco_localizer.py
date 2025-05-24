from sensor_msgs.msg import CameraInfo

import cv2
import math
import time
import numpy as np

class ArucoDetector:
    """
     At this level, everything is in pixels
    """

    def __init__(self,):
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters =  cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)


    def detect_arucos(self, image, ids_to_find=None):
        """
        None means all ids.
        """

        # Detect ArUco markers in the image
        poses = []
        ids = []
        markerCorners, markerIds, _ = self.aruco_detector.detectMarkers(image)

        if markerIds is None:
            return poses, ids
        
        for i, markerId in enumerate(markerIds):
            if ids_to_find is not None and markerId not in ids_to_find:
                continue

            # Get the corners of the marker
            corners = markerCorners[i][0]

            # Calculate the center of the marker
            center_x_1 = (corners[0][0] + corners[2][0]) / 2
            center_y_1 = (corners[0][1] + corners[2][1]) / 2

            center_x_2 = (corners[1][0] + corners[3][0]) / 2
            center_y_2 = (corners[1][1] + corners[3][1]) / 2

            center_x = int((center_x_1 + center_x_2)/2.)
            center_y = int((center_y_1 + center_y_2)/2.)

            # Calculate the angle of the marker
            angle = math.atan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0])
            poses.append([center_x, center_y, angle])
            ids.append(markerId[0])

        return poses, ids



class Visualizer:
    def __init__(self):
        self.time_last_call = time.time()
    

    def make_viz(self, image_source, detect_success: bool, detected_pose: list[float, float, float] = None): 

        if image_source is None:
            return None
        
        image = image_source.copy()
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        
        self.draw_fps(image)

        if not detect_success:
            return image
        
        self.draw_2D_axis(image, detected_pose[:2], detected_pose[2])

        return image


    def draw_fps(self, image):
        # compute fps
        fps = 1 / (time.time() - self.time_last_call)
        self.time_last_call = time.time()
        # draw fps
        cv2.putText(image, f"FPS: {fps:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    

    def draw_2D_axis(self, image, pos_pxls, angle):
        
        cv2.circle(image, pos_pxls, 10, (255,0,0), -1)

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
        cv2.line(image, tuple(pos_pxls), tuple(x_axis_rot), (0,255,0), 2)
        cv2.line(image, tuple(pos_pxls), tuple(y_axis_rot), (0,0,255), 2)
        
        return image
        