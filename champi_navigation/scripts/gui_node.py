#!/usr/bin/env python3

from icecream import ic
import cv2

from champi_navigation.rviz_img_displayer import RvizImageDisplayer

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class GuiV2(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.rviz_img_displayer = RvizImageDisplayer(self, 200., 3., 2., "odom")

        background_img_path = get_package_share_directory('champi_navigation') + '/resources/table.png'
        self.rviz_img_displayer.load_background_img(background_img_path)
    
        self.path_sub = self.create_subscription(Path, '/cmd_path', self.path_callback, 10)

        self.timer = self.create_timer(timer_period_sec=0.1,
                                    callback=self.callback)
        
        self.poses_robot = None

    def path_callback(self, msg):
        self.poses = msg.poses

    def draw_goal_poses(self):

        if len(self.poses) == 0:
            return
        
        # compute positions in pixels
        positions = [(pose.pose.position.x, pose.pose.position.y) for pose in self.poses]

        positions_px = self.rviz_img_displayer.m_to_pxl(positions)
        # draw lines
        for i in range(len(positions_px)-1):
            cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
    
    def callback(self):
        self.draw_goal_poses()
        self.rviz_img_displayer.update()
        

def main(args=None):
    rclpy.init(args=args)
    gui_node = GuiV2()
    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()