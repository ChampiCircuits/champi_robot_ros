#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_point
import time

from math import cos, sin, atan2

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
        self.x_min_box = self.declare_parameter('crop_box.x_min',rclpy.Parameter.Type.DOUBLE).value
        self.x_max_box = self.declare_parameter('crop_box.x_max', rclpy.Parameter.Type.DOUBLE).value
        self.y_min_box = self.declare_parameter('crop_box.y_min', rclpy.Parameter.Type.DOUBLE).value
        self.y_max_box = self.declare_parameter('crop_box.y_max', rclpy.Parameter.Type.DOUBLE).value

        # Print crop box parameters
        self.get_logger().info('Crop box parameters: x_min={0}, x_max={1}, y_min={2}, y_max={3}'.format(self.x_min_box,
                                                                                                        self.x_max_box,
                                                                                                        self.y_min_box,
                                                                                                        self.y_max_box))

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

        # Compute dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_measurement_time).nanoseconds / 1e9
        self.last_measurement_time = current_time


        points = []
        for i in range(len(self.scan_msg.ranges)):
            angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment

            if self.scan_msg.ranges[i] > 5.0 or self.scan_msg.ranges[i] < 0.1:
                continue

            x = self.scan_msg.ranges[i] * cos(angle)
            y = self.scan_msg.ranges[i] * sin(angle)
            point_stamped = PointStamped()
            point_stamped.point = Point(x=x, y=y, z=0.)
            point_stamped.header.frame_id = 'base_laser'
            point_stamped.header.stamp.sec = 0
            point_stamped.header.stamp.nanosec = 0
            try:
                transform = self.tf_buffer.lookup_transform('odom',
                                                            'base_laser',
                                                            point_stamped.header.stamp)
                transformed_point = do_transform_point(point_stamped, transform)

                if self.is_point_in_box(transformed_point.point):
                    points.append(transformed_point.point)

            except Exception as ex:
                self.get_logger().warn('Could not transform laser scan point: {0}'.format(str(ex)))
                return


        if len(points) > 0:
            center_of_mass = self.get_center_of_mass(points)

            enemy_odom_msg = Odometry()
            enemy_odom_msg.pose.pose.position = center_of_mass
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
                enemy_yaw = atan2(center_of_mass.y - self.prev_enemy_odom_msg.pose.pose.position.y, center_of_mass.x - self.prev_enemy_odom_msg.pose.pose.position.x)
                enemy_odom_msg.pose.pose.orientation.x = 0.
                enemy_odom_msg.pose.pose.orientation.y = 0.
                enemy_odom_msg.pose.pose.orientation.z = sin(enemy_yaw/2)
                enemy_odom_msg.pose.pose.orientation.w = cos(enemy_yaw/2)

                # Compute enemy velocity (in it's local frame)
                dist = ((center_of_mass.x - self.prev_enemy_odom_msg.pose.pose.position.x) ** 2 + (center_of_mass.y - self.prev_enemy_odom_msg.pose.pose.position.y) ** 2) ** 0.5
                enemy_odom_msg.twist.twist.linear.x = dist / dt

            self.prev_enemy_odom_msg = enemy_odom_msg

            self.enemy_pos_pub.publish(enemy_odom_msg)
            
        
        self.get_logger().info(f'Enemy Tracker loop took {time.time()-t_start}s')



    def is_point_in_box(self, point):
        return self.x_min_box <= point.x <= self.x_max_box and self.y_min_box <= point.y <= self.y_max_box

    def get_center_of_mass(self, points):
        x_sum = 0.
        y_sum = 0.
        for point in points:
            x_sum += point.x
            y_sum += point.y
        return Point(x=x_sum/len(points), y=y_sum/len(points))

    def is_steady(self, point):
        return (self.prev_enemy_odom_msg is not None and abs(point.x - self.prev_enemy_odom_msg.pose.pose.position.x) < 0.02
                and abs(point.y - self.prev_enemy_odom_msg.pose.pose.position.y) < 0.02)

def main(args=None):
    rclpy.init(args=args)
    laserscan_to_odom_node = EnemyTracker()
    rclpy.spin(laserscan_to_odom_node)
    laserscan_to_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()