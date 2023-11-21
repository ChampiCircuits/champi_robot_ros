import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2
import numpy as np
import matplotlib.pyplot as plt
import time


class VisualLocalizationNode(Node):

    def __init__(self):
        super().__init__('visual_loc')

        self.cv_bridge = CvBridge()

        self.curent_image = None
        self.time_last_image = time.time()

        self.cam_to_bird_view_transform = np.array([[ 3.43601896e+00,  1.20853081e+01, -1.66943128e+02],
                                                    [-8.34050674e-14,  2.14454976e+01,  1.02369668e+02],
                                                    [-6.19931052e-17,  1.30331754e-02,  1.00000000e+00]])
        
        self.pos_cam_in_bird_view_pxls = np.array([927, 1462])


        self.subscription = self.create_subscription(
            Image,
            '/champi/sensors/camera_1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # convert to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        self.curent_image = cv_image

        # get bird view
        bird_view = self.get_bird_view()

        tmp = self.draw_2D_axis(bird_view, self.pos_cam_in_bird_view_pxls, 0)

        tmp = cv2.resize(tmp, (0,0), fx=0.3, fy=0.3)

        self.draw_fps(tmp)

        cv2.imshow("Image window", tmp)
        # add waitKey for video to display
        cv2.waitKey(1)





    def get_bird_view(self):
        # get bird view
        bird_view = cv2.warpPerspective(self.curent_image, self.cam_to_bird_view_transform, (1800,1600))

        return bird_view


    def draw_2D_axis(self, image, pos_pxls, angle):
        img = image.copy()
        cv2.circle(img, tuple(pos_pxls), 10, (255,0,0), -1)
        # draw x axis
        cv2.line(img, tuple(pos_pxls), tuple(pos_pxls + np.array([100,0])), (0,255,0), 5)
        # draw y axis
        cv2.line(img, tuple(pos_pxls), tuple(pos_pxls + np.array([0,100])), (0, 0,255), 5)
        return img
    

    def draw_fps(self, image):
            # compute fps
        fps = 1 / (time.time() - self.time_last_image)
        self.time_last_image = time.time()

        # draw fps
        cv2.putText(image, f"FPS: {fps:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)



    


def main(args=None):
    rclpy.init(args=args)

    visual_localization = VisualLocalizationNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()