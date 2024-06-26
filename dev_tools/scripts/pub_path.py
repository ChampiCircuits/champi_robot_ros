import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import numpy as np

def generate_ellipse(a, b, h=0, k=0, num_points=100):
    t = np.linspace(0, 2*np.pi, num_points)
    x = h + a * np.cos(t)
    y = k + b * np.sin(t)
    return np.column_stack((x, y))

ellipse_points = generate_ellipse(a=2, b=1, h=1, k=1, num_points=20)


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, '/cmd_path', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define your list of poses here. Each pose is a list of [x, y, theta].
        # self.poses = [[0.0, 0.0, 0.0],
        #               [1.0, 0.0, 1.57],
        #               [2.0, 1.0, -1.57]]
        self.poses = np.column_stack((ellipse_points, np.zeros(ellipse_points.shape[0])))

    def timer_callback(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'  # or whatever frame you're using

        for pose in self.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = 0.
            pose_stamped.pose.orientation.x = 0.
            pose_stamped.pose.orientation.y = 0.
            pose_stamped.pose.orientation.z = sin(pose[2] / 2)
            pose_stamped.pose.orientation.w = cos(pose[2] / 2)
            path.poses.append(pose_stamped)

        self.publisher_.publish(path)
        # self.get_logger().info('Publishing a path')


def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()