from sensor_msgs.msg import CameraInfo

import cv2
import math

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
            center_x = int((corners[0][0] + corners[2][0]) / 2)
            center_y = int((corners[0][1] + corners[2][1]) / 2)
            # Calculate the angle of the marker
            angle = math.atan2(corners[1][1] - corners[0][1], corners[1][0] - corners[0][0])
            poses.append([center_x, center_y, angle])
            ids.append(markerId[0])

        return poses, ids

