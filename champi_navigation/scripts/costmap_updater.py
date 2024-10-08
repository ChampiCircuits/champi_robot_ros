#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from icecream import ic
import time
from rclpy.executors import ExternalShutdownException


class ImageToCostmapNode(Node):
    def __init__(self):
        super().__init__('image_to_costmap_node')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'costmap', 10)
        self.bridge = CvBridge()
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Parameter for grid width and height (meters) and resolution (m/pixel)
        self.grid_width = self.declare_parameter('grid_width', rclpy.Parameter.Type.DOUBLE).value
        self.grid_height = self.declare_parameter('grid_height', rclpy.Parameter.Type.DOUBLE).value
        self.resolution = self.declare_parameter('grid_resolution', rclpy.Parameter.Type.DOUBLE).value

        self.robot_radius = self.declare_parameter('robot_radius', rclpy.Parameter.Type.DOUBLE).value
        self.enemy_robot_radius = self.declare_parameter('enemy_robot_radius', rclpy.Parameter.Type.DOUBLE).value

        self.enemy_prediction_time = self.declare_parameter('enemy_pos_prediction_time', rclpy.Parameter.Type.DOUBLE).value

        # Create a black image with the specified width and height
        self.static_layer_img = np.zeros((int(self.grid_height / self.resolution), int(self.grid_width / self.resolution)), np.uint8)

        # Draw borders (robot radius)
        self.static_layer_img[0:int(self.robot_radius / self.resolution), :] = 100
        self.static_layer_img[-int(self.robot_radius / self.resolution):, :] = 100
        self.static_layer_img[:, 0:int(self.robot_radius / self.resolution)] = 100
        self.static_layer_img[:, -int(self.robot_radius / self.resolution):] = 100

        self.obstacle_layer_img = np.zeros((int(self.grid_height / self.resolution), int(self.grid_width / self.resolution)), np.uint8)


        # Subscribe to the enemy position
        self.enemy_position = None
        self.enemy_position_last_time = None
        self.enemy_position_sub = self.create_subscription(Odometry, '/enemy_pose', self.enemy_position_callback, 10)


    def enemy_position_callback(self, msg):
        self.enemy_position = msg
        self.enemy_position_last_time = time.time()
        

    def timer_callback(self):

        if self.enemy_position is not None and time.time() - self.enemy_position_last_time > 1.:
           self.clear_obstacle_layer()
           self.enemy_position = None 

        if self.enemy_position is not None:        
            self.clear_obstacle_layer()

            self.draw_enemy_robot(self.obstacle_layer_img, self.enemy_position.pose.pose.position.x, self.enemy_position.pose.pose.position.y)

            if self.enemy_prediction_time > 0:
                x_pred, y_pred = self.predict_enemy_pos(self.enemy_prediction_time)

                self.draw_enemy_robot(self.obstacle_layer_img, x_pred, y_pred)

        occupancy_img = self.combine_layers()

        occupancy_grid_msg = self.image_to_occupancy_grid(occupancy_img)
        self.publisher_.publish(occupancy_grid_msg)

    def clear_obstacle_layer(self):
        self.obstacle_layer_img = np.zeros((int(self.grid_height / self.resolution), int(self.grid_width / self.resolution)), np.uint8)

    def combine_layers(self):
        # Sum the two layers and clip the values to 100
        occupancy_img = np.clip(self.static_layer_img + self.obstacle_layer_img, 0, 100)
        return occupancy_img

    def image_to_occupancy_grid(self, img):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = img.shape[1]
        occupancy_grid.info.height = img.shape[0]
        # ic((255 - 128 - img.flatten()).tolist())
        occupancy_grid.data = (img.flatten()).tolist()  # invert colors and flatten the image
        return occupancy_grid


    def draw_enemy_robot(self, img, x, y):
        enemy_x = int(round(x / self.resolution))
        enemy_y = int(round(y / self.resolution))
        radius = int(np.ceil((self.enemy_robot_radius + self.robot_radius) / self.resolution))

        # Draw enemy robot
        cv2.circle(img, (enemy_x, enemy_y), radius, 100, -1)

    def predict_enemy_pos(self, dt):

        # self.enemy_position.pose.pose.position.x is the speed of the enemy robot in its own frame.

        theta = 2 * np.arctan2(self.enemy_position.pose.pose.orientation.z, self.enemy_position.pose.pose.orientation.w)

        x_pred = self.enemy_position.pose.pose.position.x + dt * np.cos(theta) * self.enemy_position.twist.twist.linear.x
        y_pred = self.enemy_position.pose.pose.position.y + dt * np.sin(theta) * self.enemy_position.twist.twist.linear.x

        return x_pred, y_pred


def main(args=None):
    rclpy.init(args=args)

    node = ImageToCostmapNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()