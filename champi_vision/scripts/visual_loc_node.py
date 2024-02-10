#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import os


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

        ref_img_path = get_package_share_directory('champi_vision') + '/ressources/images/ref_img.png'
        self.ref_img = self.load_ref_image(ref_img_path)
        self.pxl_to_m_ref = self.ref_img.shape[1]/3.0

        # detection / matching init
        self.sift_detector = cv2.SIFT_create()
        self.ref_kp, self.ref_des =  self.sift_detector.detectAndCompute(self.ref_img, None)

        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        self.flann_matcher = cv2.FlannBasedMatcher(index_params, search_params)


        self.subscription = self.create_subscription(
            Image,
            '/champi/sensors/camera_1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # convert to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        self.curent_image = cv_image

        # get bird view
        bird_view = self.get_bird_view()

        # get homography
        ok, M, matchesMask, kp1, kp2, good = self.get_homography_bird_to_ref(bird_view)

        if not ok:
            print("no homography found")
            return

        angle = self.get_angle(M)

        pos_cam_in_ref_pxls = np.matmul(M, np.append(self.pos_cam_in_bird_view_pxls, 1))
        pos_cam_in_ref_pxls = pos_cam_in_ref_pxls[:2] / pos_cam_in_ref_pxls[2]
        pos_cam_in_ref_pxls = pos_cam_in_ref_pxls.astype(int)

        pos_cam_in_ref_m = pos_cam_in_ref_pxls / self.pxl_to_m_ref

        print("pos / angle: ", pos_cam_in_ref_m, angle)

        self.visualization(bird_view, M, good, kp2)


        

        # if ok:
        #     # draw matches
        #     matches_img = self.draw_matches(bird_view, kp1, kp2, matchesMask, good)
        #     tmp = cv2.resize(matches_img, (0,0), fx=0.3, fy=0.3)
        #     cv2.imshow("Matches", tmp)
        #     cv2.waitKey(1)

        # tmp = self.draw_2D_axis(bird_view, self.pos_cam_in_bird_view_pxls, 0)

        # tmp = cv2.resize(tmp, (0,0), fx=0.3, fy=0.3)

        # self.draw_fps(tmp)

        # cv2.imshow("Image window", tmp)
        # # add waitKey for video to display
        # cv2.waitKey(1)





    def get_bird_view(self):
        # get bird view
        bird_view = cv2.warpPerspective(self.curent_image, self.cam_to_bird_view_transform, (1800,1600))

        return bird_view


    def draw_2D_axis(self, image, pos_pxls, angle):
        img = image.copy()
        cv2.circle(img, tuple(pos_pxls), 10, (255,0,0), -1)

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
        cv2.line(img, tuple(pos_pxls), tuple(x_axis_rot), (0,255,0), 2)
        cv2.line(img, tuple(pos_pxls), tuple(y_axis_rot), (0,0,255), 2)
        return img
    

    def draw_fps(self, image):
            # compute fps
        fps = 1 / (time.time() - self.time_last_image)
        self.time_last_image = time.time()

        # draw fps
        cv2.putText(image, f"FPS: {fps:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    

    def load_ref_image(self, path):
        image = cv2.imread(path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image
    

    def get_homography_bird_to_ref(self, bird_view):

        MIN_MATCH_COUNT = 10

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift_detector.detectAndCompute(bird_view, None)
        
        matches = self.flann_matcher.knnMatch(des1, self.ref_des, k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        
        M=None
        ok = False
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.ref_kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w = bird_view.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            ok = True
        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None
        
        return ok, M, matchesMask, kp1, self.ref_kp, good

    def draw_matches(self, bird_view, kp1, kp2, matchesMask, good):

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
        
        img_ret = cv2.drawMatches(bird_view,kp1,self.ref_img,kp2,good,None,**draw_params)

        return img_ret
    
    def get_angle(self, M):
        # get angle from homography matrix
        angle = np.arctan2(M[1,0], M[0,0])
        return angle
    
    
    def visualization(self, bird_view, M, good, kp2):

        bird_view_in_ref = cv2.warpPerspective(bird_view, M, self.ref_img.shape[::-1])
        added_image = cv2.addWeighted(bird_view_in_ref,0.5,self.ref_img,0.5,0)

        added_image = cv2.cvtColor(added_image, cv2.COLOR_GRAY2BGR)

        good_matches_poses = []
        for m in good:
            good_matches_poses.append(kp2[m.trainIdx].pt)

        good_matches_poses = np.array(good_matches_poses)

        # draw matches on added_image
        for pose in good_matches_poses:
            cv2.circle(added_image, tuple(pose.astype(int)), 3, (0,255, 0), -1)
        
        # draw origin cam
        pos_cam_in_ref_pxls = np.matmul(M, np.append(self.pos_cam_in_bird_view_pxls, 1))
        pos_cam_in_ref_pxls = pos_cam_in_ref_pxls[:2] / pos_cam_in_ref_pxls[2]
        pos_cam_in_ref_pxls = pos_cam_in_ref_pxls.astype(int)
        angle = self.get_angle(M)
        added_image = self.draw_2D_axis(added_image, pos_cam_in_ref_pxls, angle)

        # draw origin ref
        added_image = self.draw_2D_axis(added_image, np.array([0,0]), 0)
        
        self.draw_fps(added_image)


        cv2.imshow("Matches", added_image)
        cv2.waitKey(1)





    


def main(args=None):
    rclpy.init(args=args)

    visual_localization = VisualLocalizationNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()