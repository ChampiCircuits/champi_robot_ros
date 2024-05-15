#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from tf2_geometry_msgs import do_transform_point

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
        self.enemy_pos_pub = self.create_publisher(PoseStamped, '/enemy_pose', 10)


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

        self.prev_enemy_pose = None

    def scan_callback(self, msg):
        points = []
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment

            if(msg.ranges[i] > 30.0):
                continue

            x = msg.ranges[i] * cos(angle)
            y = msg.ranges[i] * sin(angle)
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

            enemy_pose_msg = PoseStamped()
            enemy_pose_msg.pose.position = center_of_mass
            enemy_pose_msg.header.frame_id = 'odom'
            enemy_pose_msg.header.stamp = self.get_clock().now().to_msg()

            if self.prev_enemy_pose is None:
                enemy_pose_msg.pose.orientation.x = 0.
                enemy_pose_msg.pose.orientation.y = 0.
                enemy_pose_msg.pose.orientation.z = 0.
                enemy_pose_msg.pose.orientation.w = 1.

            elif self.is_steady(center_of_mass):
                enemy_pose_msg.pose.orientation = self.prev_enemy_pose.pose.orientation

            else:
                enemy_yaw = atan2(center_of_mass.y - self.prev_enemy_pose.pose.position.y, center_of_mass.x - self.prev_enemy_pose.pose.position.x)
                enemy_pose_msg.pose.orientation.x = 0.
                enemy_pose_msg.pose.orientation.y = 0.
                enemy_pose_msg.pose.orientation.z = sin(enemy_yaw/2)
                enemy_pose_msg.pose.orientation.w = cos(enemy_yaw/2)


            self.prev_enemy_pose = enemy_pose_msg

            self.enemy_pos_pub.publish(enemy_pose_msg)



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
        return (self.prev_enemy_pose is not None and abs(point.x - self.prev_enemy_pose.pose.position.x) < 0.01
                and abs(point.y - self.prev_enemy_pose.pose.position.y) < 0.01)

def main(args=None):
    rclpy.init(args=args)
    laserscan_to_odom_node = EnemyTracker()
    rclpy.spin(laserscan_to_odom_node)
    laserscan_to_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()