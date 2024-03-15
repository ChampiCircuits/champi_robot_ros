#!/usr/bin/env python3

from icecream import ic
import cv2

from champi_navigation.rviz_img_displayer import RvizImageDisplayer

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from champi_navigation.world_state import OpponentRobotObject

class GuiV2():
    def __init__(self, node):
        # super().__init__('gui_node')

        self.rviz_img_displayer = RvizImageDisplayer(node, 200., 3., 2., "odom")

        background_img_path = get_package_share_directory('champi_navigation') + '/resources/table.png'
        self.rviz_img_displayer.load_background_img(background_img_path)
    
        self.path_sub = node.create_subscription(Path, '/cmd_path', self.path_callback, 10)

        # self.timer = self.create_timer(timer_period_sec=0.1,
        #                             callback=self.callback)
        
        self.poses= None
        self.positions_to_save = []

        self.lines_to_draw = []
        self.opponent_to_draw = None

    def path_callback(self, msg):
        self.poses = msg.poses

    def add_lines_to_draw(self, lines):
        self.lines_to_draw = lines
        # for line in lines:
        #     print(line)
        # print("\n")

    def draw_lines(self):
        for line in self.lines_to_draw:
            line = self.rviz_img_displayer.m_to_pxl(line)
            cv2.line(self.rviz_img_displayer.get_img(), line[0], line[1], (255, 255, 0), 2)

    def draw_opponent(self):
        if self.opponent_to_draw is not None:
            opponent_poly = self.opponent_to_draw.expanded_poly
            opponent_poly = [(p[0], p[1]) for p in opponent_poly.exterior.coords]
            opponent_poly = self.rviz_img_displayer.m_to_pxl(opponent_poly)
            cv2.polylines(self.rviz_img_displayer.get_img(), [opponent_poly], True, (0, 255, 255), 2)

            opponent_poly_offset = self.opponent_to_draw.expanded_poly2
            opponent_poly_offset = [(p[0], p[1]) for p in opponent_poly_offset.exterior.coords]
            opponent_poly_offset = self.rviz_img_displayer.m_to_pxl(opponent_poly_offset)
            cv2.polylines(self.rviz_img_displayer.get_img(), [opponent_poly_offset], True, (0, 255, 255), 4)


    def draw_goal_poses(self):

        if self.poses == None or len(self.poses) == 0:
            return
        
        # compute positions in pixels
        positions = [(pose.pose.position.x, pose.pose.position.y) for pose in self.poses]

        positions_px = self.rviz_img_displayer.m_to_pxl(positions)
        for p in positions_px[1:]:
            self.positions_to_save.append(p)
        # draw lines
        # for i in range(len(positions_px)-1):
        #     cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
        for i in range(len(self.positions_to_save)-1):
        # for i in range(len(self.positions_to_save)-1):
            # cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
            # cv2.line(self.rviz_img_displayer.get_img(), tuple(self.positions_to_save[i]), tuple(self.positions_to_save[i+1]), (0, 0, 255), 2)
            # draw circles
            cv2.circle(self.rviz_img_displayer.get_img(), tuple(self.positions_to_save[i]), 3, (0, 0, 255), 3)
    
    def callback(self):
        self.draw_opponent()
        self.draw_goal_poses()
        self.draw_lines()
        self.rviz_img_displayer.update()
        

# def main(args=None):
#     rclpy.init(args=args)
#     gui_node = GuiV2()
#     rclpy.spin(gui_node)
#     gui_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()