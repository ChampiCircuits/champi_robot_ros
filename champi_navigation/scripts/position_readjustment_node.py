#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from math import atan2, pi, sin, cos

# distances between lasers
DIST_0_1 = 0.2 #m
DIST_2_3 = 0.2 #m
DIST_LASER_TO_CENTER = 0.1 # m not direct distance, but either only x or y distance


# toward X -
# ^     ^
# |     |
# 0     1
#         \
#           2 ->
#center 
#           3 -> # toward 

class PositionReadjustmentNode(Node):

    def __init__(self):
        super().__init__('position_readjustment_node')
        get_logger('rclpy').info(f"\tLaunching Position Readjustment NODE...")

        self.pose_publisher_x = self.create_publisher(PoseWithCovarianceStamped, '/pose/readjustment_x', 10)
        self.pose_publisher_y = self.create_publisher(PoseWithCovarianceStamped, '/pose/readjustment_y', 10)

        # TODO les mesures lasers sont prétraitées par la STM (cad que si les valeurs sont aberrantes=trop grandes, on recoit -1)
        self.lasers_measurements_sub = self.create_subscription(Float32MultiArray, '/lasers_distances', self.lasers_measurements_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.loop_period = self.declare_parameter('position_readjustment_loop_period', rclpy.Parameter.Type.DOUBLE).value

        self.last_robot_pose: Pose = None
        self.last_laser0_dist: float = None
        self.last_laser1_dist: float = None
        self.last_laser2_dist: float = None
        self.last_laser3_dist: float = None


        get_logger('rclpy').info(f"\tPosition Readjustment NODE launched! loop period {self.loop_period}")
        self.timer = self.create_timer(self.loop_period, self.loop_spin_once)

        get_logger('rclpy').info("\tPosition Readjustment NODE launched!")

    # ==================================== ROS2 Callbacks ==========================================
    def loop_spin_once(self):
        if not self.last_laser0_dist or not self.last_robot_pose: # first measurements not received yet
            return
        
        # traitement des distances

        # FRONT WALL
        if self.last_laser0_dist == -1 or self.last_laser1_dist == -1:
            self.get_logger().warn(f'no dist for laser 0 or 1: {self.last_laser0_dist, self.last_laser1_dist}')
        else:
            diff_front = self.last_laser1_dist - self.last_laser0_dist
            angle_of_front_wall_deg = atan2(diff_front, DIST_0_1) * 180 / pi + 90
            distance_to_front_wall = (self.last_laser0_dist + self.last_laser1_dist) / 2.

        # RIGHT WALL
        if self.last_laser2_dist == -1 or self.last_laser3_dist == -1:
            self.get_logger().warn(f'no dist for laser 0 or 1: {self.last_laser2_dist, self.last_laser3_dist}')
        else:
            diff_right = abs(self.last_laser2_dist - self.last_laser3_dist) # TODO verif calcs
            angle_of_right_wall_deg = atan2(diff_right, DIST_2_3) * 180 / pi
            distance_to_right_wall = (self.last_laser2_dist + self.last_laser3_dist) / 2.

        # TODO si on utilise pas l'angle 2 lasers suffisent en tout
        robot_pose = PoseWithCovarianceStamped()
        robot_pose.header.frame_id = 'odom'
        # robot_pose.pose.pose.orientation = self.last_robot_pose.orientation
        # TODO faire le cas où on est pas dans un coin, et où n'a pas de mesure
        # TODO pareil si on est sur un bord

        # on connait la position actuelle du robot
        # recherche du cadran :
        ############################# +X
        #   3                    2   #
        #                            #
        #                            #
        #                            #
        #   4                    1     
#    +Y ###########################  Origin

        pos = self.last_robot_pose.position
        yaw_robot_deg = 2*atan2(self.last_robot_pose.orientation.z, self.last_robot_pose.orientation.w) * 180. / pi

        robot_pose.pose.pose.orientation.z = sin(angle_of_front_wall_deg*pi/180 / 2)
        robot_pose.pose.pose.orientation.w = cos(angle_of_front_wall_deg*pi/180 / 2)

        # if pos.x < 1. and pos.y < 1.5: #1
        #     if yaw_robot_deg < -170 or yaw_robot_deg > 170: # check that robot is in right orientation
        #         if 
        #             robot_pose.pose.pose.position.x = 
        #         if 
        #             robot_pose.pose.pose.position.y = 

        # elif pos.x > 1. and pos.y < 1.5: # 2
        get_logger('rclpy').warn(f"{pos.x}\t{pos.y}\t{yaw_robot_deg}\t\t\t{angle_of_front_wall_deg}")
        if pos.x > 1. and pos.y < 1.5: # 2
            # if -110 < yaw_robot_deg < -80: # check that robot is in right orientation
            if distance_to_front_wall:
                robot_pose.pose.pose.position.x = 2.0 - distance_to_right_wall - DIST_LASER_TO_CENTER
            if distance_to_right_wall:
                robot_pose.pose.pose.position.y = 0.0 + distance_to_front_wall + DIST_LASER_TO_CENTER
            get_logger('rclpy').warn("making readjustment")

        # elif pos.x > 1. and pos.y > 1.5: # 3
        #     if -10 < yaw_robot_deg < 10: # check that robot is in right orientation
        #         if 
        #             robot_pose.pose.pose.position.x = 
        #         if         

        # elif pos.x < 1. and pos.y > 1.5: # 4
        #     if 80 < yaw_robot_deg < 110: # check that robot is in right orientation
        #         if 
        #             robot_pose.pose.pose.position.x = 
        #         if 


        self.pose_publisher_x.publish(robot_pose)
        self.pose_publisher_y.publish(robot_pose)

        # get_logger('rclpy').warn("making readjustment")

    def lasers_measurements_callback(self, msg: Float32MultiArray):
        self.last_laser0_dist = msg.data[0]/1000.0
        self.last_laser1_dist = msg.data[1]/1000.0
        self.last_laser2_dist = msg.data[2]/1000.0
        self.last_laser3_dist = msg.data[3]/1000.0

        # TODO quickfix
        self.last_laser3_dist = self.last_laser2_dist

    def odom_callback(self, msg:Odometry):
        self.last_robot_pose = msg.pose.pose

    # ====================================== Utils ==========================================


def main(args=None):
    rclpy.init(args=args)

    node = PositionReadjustmentNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
