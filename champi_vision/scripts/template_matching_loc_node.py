#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2, math
import numpy as np
import matplotlib.pyplot as plt
import time
import os

from scipy.spatial.transform import Rotation as R

import champi_vision.bird_view as bv

from icecream import ic

from collections import Counter


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}"

class VisualLocalizationNode(Node):

    def __init__(self):
        super().__init__('visual_loc')

        # Parameters
        self.enable_viz = True
        self.angle_offset = 0.



        # transformation matrix between base_link and camera using tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Map to undistort the image
        self.map1 = None
        self.map2 = None

        self.img_viz = None
        self.cv_bridge = CvBridge()
        self.curent_image = None
        self.time_last_image = time.time()
        self.pos_cam_in_bird_view_pxls = None
        self.bird_view = None
        self.robot_pose = None

        self.result_init_img = None

        self.angle_initialized = False

        self.set_pose_done = False

        self.set_pose_client = self.create_client(SetPose, '/set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.future_set_pose_response = None


        ref_img_path = get_package_share_directory('champi_vision') + '/ressources/images/ref_img.png'
        self.ref_img = self.load_ref_image(ref_img_path)

        self.pxl_to_m_ref = self.ref_img.shape[1]/3.0

        self.borders_offsets = [150, 360]

        self.ref_img = cv2.copyMakeBorder(self.ref_img, self.borders_offsets[1], self.borders_offsets[1],
                                          self.borders_offsets[0], self.borders_offsets[0],
                                          cv2.BORDER_CONSTANT, value=[0, 0, 0])

        # handle camera info
        self.camera_info = None
        self.subscription_cam_info = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.callback_cam_info,
            10)


        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning


        self.sub_odom = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)


        self.pub_odom = self.create_publisher(Odometry, '/odometry/visual', 10)




    def init_bird_view(self):
        transform = None
        try:
            # get transform, which is what we need to wait for
            transform = self.tf_buffer.lookup_transform('base_link', 'camera',rclpy.time.Time().to_msg()) # todo use camera info

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
            K = np.array(self.camera_info.k).reshape(3,3)
            self.bird_view = bv.BirdView(K, transform_mtx, (0.27, -0.17), (0.82, 0.16), resolution=378)

            self.pos_cam_in_bird_view_pxls = self.bird_view.get_work_plane_pt_in_bird_img(np.array([0, 0, 1]))
            ic(self.pos_cam_in_bird_view_pxls)

            # compute undistortion map
            self.map1, self.map2 = cv2.initUndistortRectifyMap(K, np.array(self.camera_info.d), None, K, (640,480), cv2.CV_32FC1)

            self.get_logger().info("Node Initialized", once=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().info("waiting for tf2 transform...", once=True)
            return


    def odom_callback(self, msg):

        if not self.set_pose_done:
            return

        if self.robot_pose is None:
            self.robot_pose = Pose(0, 0, 0)

        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pose.theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        self.get_logger().info(f"Got first robot pose: {self.robot_pose}", once=True)

    def callback_cam_info(self, msg):
        if self.camera_info is None:
            self.camera_info = msg

    def image_callback(self, msg):

        # get camera info if not already
        if self.camera_info is None:
            self.get_logger().info("waiting for camera info...", once=True)
            return
        self.get_logger().info("camera info received", once=True)


        # initialize bird view if not already
        if self.bird_view is None:
            self.init_bird_view()


        # ==================================== PREPROCESSING ====================================

        # convert to cv2
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # undistort
        cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

        self.curent_image = cv_image

        # =================================== COMPUTE BIRD VIEW ===================================

        # get bird view
        bird_view_img = self.bird_view.project_img_to_bird(self.curent_image)

        # cv2.imshow("bird_view", bird_view_img)
        # cv2.waitKey(1)


        if not self.angle_initialized:

            if self.initialize_pose(bird_view_img):
                self.angle_initialized = True
            else:
                self.get_logger().info("Could not initialize")
                return

        if self.future_set_pose_response is None or not self.future_set_pose_response.done():
            self.get_logger().info("Waiting for set_pose service response...")
            return

        self.set_pose_done = True


        # if self.enable_viz:
        #     # draw initialization image
        #     if self.result_init_img is not None:
        #         cv2.imshow("Initialization", self.result_init_img)
        #         cv2.waitKey(1)

        # Wait for robot pose
        if self.robot_pose is None:
            self.get_logger().info("waiting for robot pose...", once=True)
            return
        self.get_logger().info("robot pose received", once=True)

        self.get_logger().info("Starting visual localization!", once=True)



        # # Save bird view to file
        # cv2.imwrite("bird_view.png", bird_view_img)
        # # Save predicted bird view to file
        # cv2.imwrite("predicted_bird_view.png", bv_pred)

        # ============================= COMPUTE POSE =================================

        angle = self.robot_pose.theta
        robot_pos = self.get_pos_robot(angle, bird_view_img)
        robot_pose_computed = Pose(robot_pos[1], robot_pos[0], angle)

        # Print current pose from odometry and computed pose
        self.get_logger().info(f"Robot pose (odom): {self.robot_pose}")
        self.get_logger().info(f"Robot pose (computed): {robot_pose_computed}")


        # ============================= PUBLISH ODOMETRY =================================

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = robot_pos[0]
        odom_msg.pose.pose.position.y = robot_pos[1]
        q = R.from_euler('z', angle).as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Very low covariance
        cov = 0.00001
        for i in range(36):
            if i % 7 == 0:
                odom_msg.pose.covariance[i] = cov

        self.pub_odom.publish(odom_msg)


    def initialize_pose(self, bird_view):

        # 1) Compute Angle
        angle = self.compute_angle(bird_view)
        if angle is None:
            return False

        self.get_logger().info("Angle initialized: " + str(angle) + "(" + str(angle*180/np.pi) + "°)")


        # 2) Compute Pos

        angle = -np.pi/2-angle
        robot_pos = self.get_pos_robot(angle, bird_view, vote=True)

        ic("Robot pos (init): ", robot_pos)

        # 3) Call service set_pose

        pose_init = Pose(robot_pos[1], robot_pos[0], angle+self.angle_offset) # TODO Pourquoi j'ai dû swap ??

        self.call_set_pose(pose_init)

        return True





    def call_set_pose(self, pose):

        request = SetPose.Request()
        # request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = pose.x
        request.pose.pose.pose.position.y = pose.y
        request.pose.pose.pose.position.z = 0.
        request.pose.pose.pose.orientation.x = 0.
        request.pose.pose.pose.orientation.y = 0.
        request.pose.pose.pose.orientation.z = np.sin(pose.theta/2)
        request.pose.pose.pose.orientation.w = np.cos(pose.theta/2)

        self.get_logger().info('Calling /set_pose...')
        self.future_set_pose_response = self.set_pose_client.call_async(request)

    def compute_angle(self, bird_view):

        _, img_bin = cv2.threshold(bird_view, 200, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(img_bin, 50, 150, apertureSize=3)

        # Find lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)

        if lines is None:
            return None

        # Draw lines
        if self.enable_viz:
            self.result_init_img = np.zeros_like(bird_view)
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(self.result_init_img, (x1, y1), (x2, y2), 255, 2)

        # Compute angle
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angles.append(np.arctan2(y2-y1, x2-x1))

        return np.median(angles)


    def get_rotated_ref_img(self, angle):
        rows, cols = self.ref_img.shape
        M = cv2.getRotationMatrix2D((cols/2, rows/2), np.rad2deg(angle), 1)
        ref_img_rot = cv2.warpAffine(self.ref_img, M, (cols, rows))
        return M, ref_img_rot

    def get_positions_with_template_matching(self, img, template):

        # All the 6 methods for comparison in a list
        methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                   'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

        positions = []

        for meth in methods:

            method = eval(meth)

            time_start = time.time()

            # Apply template Matching
            res = cv2.matchTemplate(img,template,method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            time_end = time.time()

            self.get_logger().info(f"fime elapsed for {meth}: {time_end - time_start}")

            # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
            if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc

            positions.append(top_left)


        return positions


    def display_template_in_image(self, img, template, pos):

        bottom_right = (pos[0] + template.shape[1], pos[1] + template.shape[0])

        img_cp = img.copy()
        cv2.rectangle(img, pos, bottom_right, 255, 2)
        img_cp[pos[1]:bottom_right[1], pos[0]:bottom_right[0]] = template

        # Resize
        img_cp = cv2.resize(img_cp, (0,0), fx=0.2, fy=0.2)

        cv2.imshow("Template in image", img_cp)
        cv2.waitKey(1)


    def pos_template_matching_to_pos_robot_m(self, M, angle, pos_template):

        # 3) Compute the pos of the robot in the ref image (in pixels)

        pos_robot_in_template = self.pos_cam_in_bird_view_pxls
        pos_robot_in_rot_ref = [pos_template[0] + pos_robot_in_template[0], pos_template[1] + pos_robot_in_template[1], 1]

        M_inv = np.linalg.inv(np.vstack([M, [0, 0, 1]]))
        pos_robot_in_ref_pxls = np.dot(M_inv, pos_robot_in_rot_ref)

        # 4) Remove border offsets and convert to meters

        translation = np.array([self.borders_offsets[0], self.borders_offsets[1], 0])
        pos_robot_in_ref_no_borders_pxls = pos_robot_in_ref_pxls - translation

        pos_robot_in_ref_m = pos_robot_in_ref_no_borders_pxls / self.pxl_to_m_ref

        return pos_robot_in_ref_m



    def get_pos_robot(self, angle_robot, bird_view, vote=False):

        # 1) Compute rotated ref image
        M, ref_img_rot = self.get_rotated_ref_img(angle_robot)


        # 2) Perform template matching to find the pos of bv in rotated ref image
        positions_bv_in_rot_ref = self.get_positions_with_template_matching(ref_img_rot, bird_view)

        # Get closest position to the current robot position

        pos_robot_in_ref_m_ret = None
        pos_bv_in_rot_ref_ret = None
        dist_min = 1000000.

        if(vote):
            counter = Counter(positions_bv_in_rot_ref).most_common(1)
            pos_bv_in_rot_ref_ret = counter[0][0]
            pos_robot_in_ref_m_ret = self.pos_template_matching_to_pos_robot_m(M, angle_robot, pos_bv_in_rot_ref_ret)

        else:
            for pos_bv_in_rot_ref in positions_bv_in_rot_ref:

                pos_bv_in_rot_ref = np.array([pos_bv_in_rot_ref[0], pos_bv_in_rot_ref[1], 1])

                robot_pos_m = self.pos_template_matching_to_pos_robot_m(M, angle_robot, pos_bv_in_rot_ref)

                dist = np.linalg.norm(np.array([robot_pos_m[0], robot_pos_m[1]]) - np.array([self.robot_pose.x, self.robot_pose.y]))

                if dist < dist_min:
                    pos_robot_in_ref_m_ret = robot_pos_m
                    pos_bv_in_rot_ref_ret = pos_bv_in_rot_ref
                    dist_min = dist

        if self.enable_viz:
            self.display_template_in_image(ref_img_rot, bird_view, pos_bv_in_rot_ref_ret[:2])

        return pos_robot_in_ref_m_ret









    def predict_bird_view(self, shape):

        """
        we want to reverse this operation:
        pos_robot_in_ref_pxls = np.matmul(M, self.pos_cam_in_bird_view_pxls)
        pos_robot_in_ref_pxls = pos_robot_in_ref_pxls[:2] / pos_robot_in_ref_pxls[2]
        pos_robot_in_ref_pxls = pos_robot_in_ref_pxls.astype(int)
        """

        # on veut la transfo

        angle = self.current_angle
        pos_robot_in_ref_m = self.current_pos
        pos_robot_in_ref_pxls = pos_robot_in_ref_m * self.pxl_to_m_ref

        angle = angle-math.pi

        t_ref_to_robot = np.array([[1.,0, -pos_robot_in_ref_pxls[1]],
                                   [0, 1., -pos_robot_in_ref_pxls[0]],
                                   [0, 0, 1]])
        r_ref_to_robot = np.array([[np.cos(angle), -np.sin(angle),0],
                                   [np.sin(angle), np.cos(angle), 0],
                                   [0, 0, 1]])
        T_ref_to_robot = r_ref_to_robot @ t_ref_to_robot


        R = np.array([[0, 1, 0],
                      [-1, 0, 0],
                      [0, 0, 1]])


        offset_x = 0.3
        offset_y = 1.

        # Matrix for translation from cam to img
        T = np.array([[1, 0, self.pos_cam_in_bird_view_pxls[0] + offset_x*self.pxl_to_m_ref],
                      [0, 1, self.pos_cam_in_bird_view_pxls[1]+ 0.15*self.pxl_to_m_ref + offset_y*self.pxl_to_m_ref],
                      [0, 0, 1]])


        T_ref_to_bv =  T_ref_to_robot @ T


        size_x = 2000
        size_y = 2000


        bird_view_img = cv2.warpAffine(self.ref_img, T_ref_to_bv[:2], (size_x, size_y))

        return bird_view_img


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


def main(args=None):
    rclpy.init(args=args)

    visual_localization = VisualLocalizationNode()

    rclpy.spin(visual_localization)

    visual_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
