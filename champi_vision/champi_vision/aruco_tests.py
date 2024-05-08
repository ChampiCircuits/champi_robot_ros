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


for i, corners in enumerate(markerCorners):
    print("corners", corners)
    print("ids", markerIds[i])
    if (markerIds[i] >= 20)