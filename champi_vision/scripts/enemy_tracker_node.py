#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_point
import time

from math import cos, sin, atan2

from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon

class EnemyTracker(Node):
    def __init__(self):
        super().__init__('enemy_tracker')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publish enemy position
        self.enemy_pos_pub = self.create_publisher(Odometry, '/enemy_pose', 10)

        # Get crop box parameters
        x_min_box = self.declare_parameter('crop_box.x_min',rclpy.Parameter.Type.DOUBLE).value
        x_max_box = self.declare_parameter('crop_box.x_max', rclpy.Parameter.Type.DOUBLE).value
        y_min_box = self.declare_parameter('crop_box.y_min', rclpy.Parameter.Type.DOUBLE).value
        y_max_box = self.declare_parameter('crop_box.y_max', rclpy.Parameter.Type.DOUBLE).value

        self.box_points = [(x_min_box, y_min_box), (x_max_box, y_min_box), (x_max_box, y_max_box), (x_min_box, y_max_box)]

        # Print crop box parameters
        self.get_logger().info('Crop box parameters: x_min={0}, x_max={1}, y_min={2}, y_max={3}'.format(x_min_box,
                                                                                                        x_max_box,
                                                                                                        y_min_box,
                                                                                                        y_max_box))

        self.prev_enemy_odom_msg = None

        self.last_measurement_time = self.get_clock().now()
        self.scan_msg = None

        self.timer = self.create_timer(0.5, self.timer_callback)


    def scan_callback(self, msg):
        self.scan_msg = msg


    def timer_callback(self):


        t_start = time.time()

        if self.scan_msg is None:
            return

        # 1) Compute dt (for speed computation)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_measurement_time).nanoseconds / 1e9
        self.last_measurement_time = current_time

        # 2) Transform bounding box from odom to base_laser frame (instead of transforming each point, for efficiency)

        transformed_box_points = []
        for point in self.box_points:
            transformed_point = self.transform_point(point, 'odom', 'base_laser')
            if transformed_point is not None:
                transformed_box_points.append(transformed_point)
            # else: we ignore the point
        
        # 3) Get points inside the bounding box

        points = []
        for i in range(len(self.scan_msg.ranges)):
            angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment

            if self.scan_msg.ranges[i] > 4.0 or self.scan_msg.ranges[i] < 0.1:
                continue

            x = self.scan_msg.ranges[i] * cos(angle)
            y = self.scan_msg.ranges[i] * sin(angle)

            if self.is_point_in_box([x, y], transformed_box_points):
                points.append([x, y])
           

        if len(points) > 0:

            # 4) Find the closest point to origin (base_laser frame)

            closest_point = min(points, key=lambda p: p[0]**2 + p[1]**2)

            # 5) Remove points that are too far from the closest point
            
            points = [point for point in points if (point[0] - closest_point[0])**2 + (point[1] - closest_point[1])**2 < 0.1**2]

            # 6) Compute center of mass of the points

            center_of_mass = self.get_center_of_mass(points)

            # Transform center of mass to odom frame
            center_of_mass = self.transform_point(closest_point, 'base_laser', 'odom')
            if center_of_mass is None:
                return
            
            # Publish enemy position (and speed)

            enemy_odom_msg = Odometry()
            enemy_odom_msg.pose.pose.position.x = center_of_mass[0]
            enemy_odom_msg.pose.pose.position.y = center_of_mass[1]
            enemy_odom_msg.pose.pose.position.z = 0.
            enemy_odom_msg.header.frame_id = 'odom'
            enemy_odom_msg.header.stamp = self.get_clock().now().to_msg()

            if self.prev_enemy_odom_msg is None:
                enemy_odom_msg.pose.pose.orientation.x = 0.
                enemy_odom_msg.pose.pose.orientation.y = 0.
                enemy_odom_msg.pose.pose.orientation.z = 0.
                enemy_odom_msg.pose.pose.orientation.w = 1.

            elif self.is_steady(center_of_mass):
                enemy_odom_msg.pose.pose.orientation = self.prev_enemy_odom_msg.pose.pose.orientation

            else:
                enemy_yaw = atan2(center_of_mass[1] - self.prev_enemy_odom_msg.pose.pose.position.y, center_of_mass[0] - self.prev_enemy_odom_msg.pose.pose.position.x)
                enemy_odom_msg.pose.pose.orientation.x = 0.
                enemy_odom_msg.pose.pose.orientation.y = 0.
                enemy_odom_msg.pose.pose.orientation.z = sin(enemy_yaw/2)
                enemy_odom_msg.pose.pose.orientation.w = cos(enemy_yaw/2)

                # Compute enemy velocity (in it's local frame)
                dist = ((center_of_mass[0] - self.prev_enemy_odom_msg.pose.pose.position.x) ** 2 + (center_of_mass[1] - self.prev_enemy_odom_msg.pose.pose.position.y) ** 2) ** 0.5
                enemy_odom_msg.twist.twist.linear.x = dist / dt

            self.prev_enemy_odom_msg = enemy_odom_msg

            self.enemy_pos_pub.publish(enemy_odom_msg)
            
        
        self.get_logger().info(f'Enemy Tracker loop took {time.time()-t_start}s. Detected {len(points)} points.')


    def is_point_in_box(self, point, poly_points):
        polygon = Polygon(poly_points)
        return polygon.contains(ShapelyPoint(point))


    def get_center_of_mass(self, points):
        x_sum = 0.
        y_sum = 0.
        for point in points:
            x_sum += point[0]
            y_sum += point[1]
        return [x_sum/len(points), y_sum/len(points)]

    def is_steady(self, point):
        return (self.prev_enemy_odom_msg is not None and abs(point[0] - self.prev_enemy_odom_msg.pose.pose.position.x) < 0.02
                and abs(point[1] - self.prev_enemy_odom_msg.pose.pose.position.y) < 0.02)
    
    def transform_point(self, point: list, parent_frame: str, child_frame: str):
        """
        Transforms a point from parent_frame to child_frame. the point is in the form of [x, y] coordinates.
        """

        point_stamped = PointStamped()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = 0.0
        point_stamped.header.frame_id = parent_frame
        point_stamped.header.stamp.sec = 0
        point_stamped.header.stamp.nanosec = 0

        try:
            transform = self.tf_buffer.lookup_transform(child_frame,
                                                        parent_frame,
                                                        point_stamped.header.stamp)
            transformed_point = do_transform_point(point_stamped, transform)
            return [transformed_point.point.x, transformed_point.point.y]

        except Exception as ex:
            self.get_logger().warn('Could not transform point: {0}'.format(str(ex)))
            return None
        

def main(args=None):
    rclpy.init(args=args)
    laserscan_to_odom_node = EnemyTracker()
    rclpy.spin(laserscan_to_odom_node)
    laserscan_to_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()