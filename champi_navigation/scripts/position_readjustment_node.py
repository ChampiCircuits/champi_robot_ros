#!/usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from math import atan2, pi

# distances between lasers
DIST_0_1 = 0.2 #m
DIST_2_3 = 0.2 #m
DIST_LASER_TO_CENTER = 0.1 # m not direct distance, but either only x or y distance

# ^     ^
# |     |
# 0     1
#         \
#           2 ->
#center 
#           3 ->

class PositionReadjustmentNode(Node):

    def __init__(self):
        super().__init__('position_readjustment_node')
        get_logger('rclpy').info(f"\tLaunching Position Readjustment NODE...")

        self.pose_publisher_x = self.create_publisher(PoseWithCovarianceStamped, '/pose/readjustment_x', 10)
        self.pose_publisher_y = self.create_publisher(PoseWithCovarianceStamped, '/pose/readjustment_y', 10)

        # TODO les mesures lasers sont prétraitées par la STM (cad que si les valeurs sont aberrantes=trop grandes, on recoit -1)
        self.lasers_measurements_sub = self.create_subscription(Float32MultiArray, '/lasers_measurements', self.lasers_measurements_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.loop_period = self.declare_parameter('position_readjustment_loop_period', rclpy.Parameter.Type.DOUBLE).value

        self.last_robot_pose: Pose
        self.last_laser0_dist: float
        self.last_laser1_dist: float
        self.last_laser2_dist: float
        self.last_laser3_dist: float


        self.timer = self.create_timer(self.loop_period, self.loop_spin_once)

        get_logger('rclpy').info("\tPosition Readjustment NODE launched!")

    # ==================================== ROS2 Callbacks ==========================================
    def loop_spin_once(self):
        if self.last_laser0_dist is None or self.last_robot_pose is None: # first measurements not received yet
            return
        
        # traitement des distances

        # FRONT WALL
        if self.last_laser0_dist == -1 or self.last_laser1_dist == -1:
            self.get_logger().warn(f'no dist for laser 0 or 1: {self.last_laser0_dist, self.last_laser1_dist}')
        else:
            diff_front = abs(self.last_laser0_dist - self.last_laser1_dist)
            angle_of_front_wall = atan2(diff_front, DIST_0_1)
            distance_to_front_wall = (self.last_laser0_dist + self.last_laser1_dist) / 2.

        # RIGHT WALL
        if self.last_laser2_dist == -1 or self.last_laser3_dist == -1:
            self.get_logger().warn(f'no dist for laser 0 or 1: {self.last_laser2_dist, self.last_laser3_dist}')
        else:
            diff_right = abs(self.last_laser2_dist - self.last_laser3_dist)
            angle_of_right_wall = atan2(diff_right, DIST_2_3)
            distance_to_right_wall = (self.last_laser2_dist + self.last_laser3_dist) / 2.

        # TODO si on utilise pas l'angle 2 lasers suffisent en tout
        robot_pose = PoseWithCovarianceStamped()

        # TODO faire le cas où on est pas dans un coin, et où n'a pas de mesure

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
        yaw_robot_deg = 2*(self.last_robot_pose.orientation.z, self.last_robot_pose.orientation.w) * 180. / pi
        if pos.x < 1. and pos.y < 1.5: #1
            if yaw_robot_deg < -170 or yaw_robot_deg > 170: # check that robot is in right orientation
                if 
                    robot_pose.pose.pose.position.x = 
                if 
                    robot_pose.pose.pose.position.y = 

        elif pos.x > 1. and pos.y < 1.5: # 2
            if -110 < yaw_robot_deg < -80: # check that robot is in right orientation
                if distance_to_front_wall:
                    robot_pose.pose.pose.position.x = 2.0 - distance_to_front_wall - DIST_LASER_TO_CENTER
                if distance_to_right_wall:
                    robot_pose.pose.pose.position.y = 0.0 + distance_to_right_wall + DIST_LASER_TO_CENTER

        elif pos.x > 1. and pos.y > 1.5: # 3
            if -10 < yaw_robot_deg < 10: # check that robot is in right orientation
                if 
                    robot_pose.pose.pose.position.x = 
                if         

        elif pos.x < 1. and pos.y > 1.5: # 4
            if 80 < yaw_robot_deg < 110: # check that robot is in right orientation
                if 
                    robot_pose.pose.pose.position.x = 
                if 



    def lasers_measurements_callback(self, msg: Float32MultiArray):
        self.last_laser0_dist = msg.data[0]
        self.last_laser1_dist = msg.data[1]
        self.last_laser2_dist = msg.data[2]
        self.last_laser3_dist = msg.data[3]

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
