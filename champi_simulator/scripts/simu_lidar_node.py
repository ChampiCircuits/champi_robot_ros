#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math
import random
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

from rclpy.executors import ExternalShutdownException

class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5Hz
        self.obstacle_position = Point(x=1.0, y=2.0, z=0.0)
        self.target_position = self.obstacle_position
        self.speed = 0.5  # m/s
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)

    def clicked_point_callback(self, msg):
        self.target_position = msg.point

    def timer_callback(self):
        # Move the obstacle towards the target position
        direction = Point(
            x=self.target_position.x - self.obstacle_position.x,
            y=self.target_position.y - self.obstacle_position.y,
            z=self.target_position.z - self.obstacle_position.z)
        distance = self.calculate_distance(direction)
        if distance > 0.1:
            scale = self.speed * 0.2 / distance  # 0.2s time step
            self.obstacle_position.x += direction.x * scale
            self.obstacle_position.y += direction.y * scale
            self.obstacle_position.z += direction.z * scale

        scan = LaserScan()
        scan.angle_min = -3.14  # -180 degrees
        scan.angle_max = 3.14   # 180 degrees
        scan.angle_increment = 6.28 / 1024  # 360 degrees / 1024 points
        scan.range_min = 0.0
        scan.range_max = 5.
        scan.ranges = [scan.range_max] * 1024  # Initialize all ranges to max
        scan.intensities = [0.] * 1024  # Initialize all ranges to max

        # obstacle_point = PointStamped()
        # obstacle_point.header.frame_id = 'odom'
        # obstacle_point.point = self.obstacle_position

        # try:
        #     transformed_point = self.tf_buffer.transform(obstacle_point, 'base_laser')
        #     angle = self.calculate_angle(transformed_point.point)
        #     index = int((angle - scan.angle_min) / scan.angle_increment)
        #     distance = self.calculate_distance(transformed_point.point)
        #     scan.ranges[index] = distance
        #     # add a 5 more points on each side of the obstacle to simulate the width of the obstacle.
        #     # Care of not going out of the array bounds
        #     for i in range(max(0, index - 5), min(index + 5, 1023)):
        #         # add random noise to the distance
        #         scan.ranges[i] = distance + random.uniform(-0.01, 0.01)

        BEACON_1_POS = PointStamped()
        BEACON_1_POS.point.x = 0.8
        BEACON_1_POS.point.y = 1.5
        BEACON_2_POS = PointStamped()
        BEACON_2_POS.point.x = 2.0
        BEACON_2_POS.point.y = 1.5
        BEACON_3_POS = PointStamped()
        BEACON_3_POS.point.x = 2.0
        BEACON_3_POS.point.y = 0.8

        R_BEACON = 0.04 #0.04
        n = 33
        uniform = 0.01 #0.01

        try:
            for beacon in [BEACON_1_POS,BEACON_2_POS,BEACON_3_POS]:
                for i in range(n): # n points per circle
                    angle = 2 * i * math.pi/n
                    p = tf2_geometry_msgs.PointStamped()
                    p.header.frame_id = 'odom'
                    p.point.x = beacon.point.x + R_BEACON*math.cos(angle)
                    p.point.y = beacon.point.y + R_BEACON*math.sin(angle)

                    transformed_point = self.tf_buffer.transform(p, 'base_laser')
                    angle = self.calculate_angle(transformed_point.point)
                    index = int((angle - scan.angle_min) / scan.angle_increment)
                    distance = self.calculate_distance(transformed_point.point)
                    
                    scan.ranges[index] = distance + random.uniform(-uniform, uniform)
                    scan.intensities[index] = 255.

        except Exception as e:
            self.get_logger().warn(message='Could not transform obstacle point: ' + str(e))

        scan.header.frame_id = 'base_laser'
        scan.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(scan)

    def calculate_angle(self, point):
        return math.atan2(point.y, point.x)

    def calculate_distance(self, point):
        return math.sqrt(point.x**2 + point.y**2)

def main(args=None):
    rclpy.init(args=args)

    node = LidarSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()