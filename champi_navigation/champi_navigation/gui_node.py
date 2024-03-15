#!/usr/bin/env python3

from icecream import ic
import cv2

from champi_navigation.rviz_img_displayer import RvizImageDisplayer

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from champi_navigation.world_state import OpponentRobotObject
from geometry_msgs.msg import PoseStamped, PoseWithCovariance

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
        self.robot_to_draw = None

    def path_callback(self, msg):
        self.poses = msg.poses

    def add_lines_to_draw(self, lines):
        self.lines_to_draw = lines

    def draw_lines(self):
        # for line in self.lines_to_draw:
        #     line = self.rviz_img_displayer.m_to_pxl(line)
        #     cv2.line(self.rviz_img_displayer.get_img(), line[0], line[1], (255, 255, 0), 2)

        for line in self.lines_to_draw:
            line = self.rviz_img_displayer.m_to_pxl(line)
            #point
            cv2.circle(self.rviz_img_displayer.get_img(), line[0], 3, (0, 0, 0), 1)

    def draw_opponent(self):
        if self.opponent_to_draw is not None:
            opponent_poly_2 = self.opponent_to_draw.expanded_poly
            opponent_poly_2 = [(p[0], p[1]) for p in opponent_poly_2.exterior.coords]
            opponent_poly_2 = self.rviz_img_displayer.m_to_pxl(opponent_poly_2)
            cv2.polylines(self.rviz_img_displayer.get_img(), [opponent_poly_2], True, (0, 255, 255), 2)

            opponent_poly_1 = self.opponent_to_draw.polygon
            opponent_poly_1 = [(p[0], p[1]) for p in opponent_poly_1.exterior.coords]
            opponent_poly_1 = self.rviz_img_displayer.m_to_pxl(opponent_poly_1)
            cv2.polylines(self.rviz_img_displayer.get_img(), [opponent_poly_1], True, (0, 255, 255), 4)

    def draw_robot(self):
        if self.robot_to_draw is not None:
            # quickfix if msg is posestamped or posewithcov
            # print(type(self.robot_to_draw.pose_stamped), type(PoseStamped()))

            if isinstance(self.robot_to_draw.pose_stamped, PoseStamped):
                print("posestamped")
                pose_with_cov = self.robot_to_draw.pose_stamped.pose
                # print(type(self.robot_to_draw.pose_stamped.pose))
                if (isinstance(self.robot_to_draw.pose_stamped.pose, PoseWithCovariance)):
                    x,y = pose_with_cov.pose.position.x, pose_with_cov.pose.position.y
                    #draw circle
                    center = self.rviz_img_displayer.m_to_pxl([(x,y)])[0]
                    OFFSET = 0.2
                    radius = self.rviz_img_displayer.m_to_pxl([(OFFSET,0)])[0][0]
                    cv2.circle(self.rviz_img_displayer.get_img(), center, radius, (0, 255, 0), 3)
            else:
                print(type(self.robot_to_draw.pose_stamped))
            #     pose_stamped = self.robot_to_draw.pose_stamped
            #     x,y = pose_stamped.pose.position.x, pose_stamped.pose.position.y
            #     #draw circle
            #     center = self.rviz_img_displayer.m_to_pxl([(x,y)])[0]
            #     OFFSET = 0.2
            #     radius = self.rviz_img_displayer.m_to_pxl([(OFFSET,0)])[0][0]
            #     cv2.circle(self.rviz_img_displayer.get_img(), center, radius, (0, 255, 0), 3)
            # else:
            #     print("pose with cov")
            #     pose_cov = self.robot_to_draw.pose_stamped
            #     x,y = pose_cov.pose.position.x, pose_cov.pose.position.y
            #     #draw circle
            #     center = self.rviz_img_displayer.m_to_pxl([(x,y)])[0]
            #     OFFSET = 0.2
            #     radius = self.rviz_img_displayer.m_to_pxl([(OFFSET,0)])[0][0]
            #     cv2.circle(self.rviz_img_displayer.get_img(), center, radius, (0, 255, 0), 3)


    def draw_goal_poses(self):

        if self.poses == None or len(self.poses) == 0:
            return
        
        # compute positions in pixels
        positions = [(pose.pose.position.x, pose.pose.position.y) for pose in self.poses]

        positions_px = self.rviz_img_displayer.m_to_pxl(positions)
        # for p in positions_px[1:]:
        #     self.positions_to_save.append(p)
        # draw lines
        for i in range(len(positions_px)-1):
        #     cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
        # for i in range(len(self.positions_to_save)-1):
        # for i in range(len(self.positions_to_save)-1):
            # cv2.line(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), tuple(positions_px[i+1]), (0, 0, 255), 2)
            # cv2.line(self.rviz_img_displayer.get_img(), tuple(self.positions_to_save[i]), tuple(self.positions_to_save[i+1]), (0, 0, 255), 2)
            # draw circles
            cv2.circle(self.rviz_img_displayer.get_img(), tuple(positions_px[i]), 3, (0, 0, 255), 3)
    
    def callback(self):
        self.draw_robot()
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