#!/usr/bin/env python3

from icecream import ic
import cv2

from champi_navigation.rviz_img_displayer import RvizImageDisplayer

from ament_index_python.packages import get_package_share_directory

# TODO est ce qu'on la garde dans le node de nav ? Elle y est seulement pour qu'on puisse dessiner sur la carte,
# mais là on a rien à dessiner
# Ca pourrait aussi être un node à part entière, qui publie juste la table et c'est tout

# je pense qu'on peut faire ca ouais

class GuiV2:
    def __init__(self, node):

        self.rviz_img_displayer = RvizImageDisplayer(node, 200., 3., 2., "odom")

        background_img_path = get_package_share_directory('champi_navigation') + '/resources/table.png'
        self.rviz_img_displayer.load_background_img(background_img_path)
    

    def draw_goal_poses(self, pose_robot, poses):

        if len(poses) == 0:
            return
        
        # compute positions in pixels
        positions = [pose_robot[:2]] + [pose[:2] for pose in poses]

        positions_px = self.rviz_img_displayer.m_to_pxl(positions)
        # draw lines
        for i in range(len(positions_px)-1):
            cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
    
    def update(self):
        self.rviz_img_displayer.update()
        
