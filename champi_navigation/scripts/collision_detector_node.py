#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')

        self.declare_parameter('vector_tolerance')
        self.declare_parameter('time_threshold')

        self.vector_tol = self.get_parameter('vector_tolerance').value
        self.time_threshold = self.get_parameter('time_threshold').value

        if None in [self.vector_tol, self.time_threshold]:
            self.get_logger().error("Strictly required parameters are missing.")
            raise ValueError("Parameters vector_tolerance and time_threshold must be set.")

        self.current_cmd_vector = np.zeros(3)
        self.discrepancy_start_time = None
        self.collision_state = False

        self.create_subscription(Twist, '/ctrl/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/odom_otos', self.odom_cb, 10)
        
        # Publisher for the boolean topic
        self.collision_pub = self.create_publisher(Bool, 'collision_detected', 10)

    def cmd_vel_cb(self, msg):
        self.current_cmd_vector = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def odom_cb(self, msg):
        actual_vector = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z
        ])

        vector_diff = np.linalg.norm(self.current_cmd_vector - actual_vector)

        if vector_diff > self.vector_tol:
            current_time = self.get_clock().now()
            
            if self.discrepancy_start_time is None:
                self.discrepancy_start_time = current_time
            else:
                elapsed_time = (current_time - self.discrepancy_start_time).nanoseconds / 1e9
                if elapsed_time > self.time_threshold:
                    if not self.collision_state:
                        self.get_logger().warn(f"COLLISION DETECTED: Discrepancy ({vector_diff:.2f})")
                    self.collision_state = True
        else:
            self.discrepancy_start_time = None
            self.collision_state = False

        # Publish 1 (True) when detected, 0 (False) when not detected
        bool_msg = Bool()
        bool_msg.data = self.collision_state
        self.collision_pub.publish(bool_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CollisionDetector()
        rclpy.spin(node)
    except ValueError as e:
        print(f"Initialization Failed: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()