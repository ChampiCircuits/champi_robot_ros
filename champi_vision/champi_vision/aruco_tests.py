import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl

import time

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# open the image "test_arcuo.png"
# frame = PIL.Image.open()
frame = cv2.imread("/home/champi/dev/ws_0/src/champi_robot_ros/champi_vision/champi_vision/test_aruco.png")


start = time.time()
# GET A FRAME

markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

end = time.time()
print("time: ",end-start)
height, width, _ = frame.shape
center_screen = (width / 2, height / 2)
print("H, W",height,width)

for i, corners in enumerate(markerCorners):
    print("corners", corners)
    print("ids", markerIds[i])
    if markerIds[i] >= 20 and markerIds[i]<=24:# arucos au sol
        center_marker = np.mean(corners[0], axis=0)

        # Position relative du centre du marqueur par rapport au centre de l'image
        relative_position = center_marker - center_screen

        # Angle du marqueur par rapport à l'axe horizontal (angle du premier côté)
        dx = corners[0][1][0] - corners[0][0][0]
        dy = corners[0][1][1] - corners[0][0][1]
        angle_rad = np.arctan2(dy, dx)  # angle en radians

        print("center_marker",center_marker)
        print("angle",angle_rad)