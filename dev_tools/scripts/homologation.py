import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time
from pid_move import *
from math import atan2

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publishers and subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, "/base_controller/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.update_robot_pose, 10)

        # PID controller parameters
        self.kp = 0.5  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.2  # Derivative gain

        # Robot pose
        self.robot_pose = None

        self.current_target_pose = None
        self.i_targets = 0

        self.stop_all = False
        self.first_time = True

    def update_robot_pose(self, msg):
        # Callback to update robot's current pose
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, 2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)]

    def listener_callback(self, msg):
        if self.first_time:
            self.start_time = time.time()
            self.last_time = time.time()
            self.first_time = False
        if self.stop_all:
            quit()
            return
        stop = False
        # Assuming the ranges array is populated
        for distance in msg.ranges:
            # print(distance)
            if distance < 0.5 and distance >0.1:
                # print(f"Distance: {distance} is greater than 0.3")
                stop = True
        if stop:
            # publish 0 cmd_vel
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)

            # si on est stopped, on décompte pas le temps
            self.get_logger().info(f"stopped")
        else:
            if self.i_vel >= len(self.vels):
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
                return
            
            self.get_logger().info(f"{self.i_vel}")
            twist = go_to_pose(self.current_target_pose, self.robot_pose)
            
            self.pub_cmd_vel.publish(twist)
            
            if self.temps_deplacement > self.vels[self.i_vel][2]:
                self.i_vel += 1
                self.start_time = time.time()
                self.temps_deplacement = 0

            if self.i_vel > len(self.vels):
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.pub_cmd_vel.publish(twist)
                self.stop_all = True


            self.temps_deplacement += time.time() - self.last_time

        self.last_time = time.time()

def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarNode()

    rclpy.spin(lidar_node)

    # Destroy the node explicitly
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()