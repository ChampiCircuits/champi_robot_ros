import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.file = open("accel_data.txt", "w")
        self.count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.average_acc_x = 0.
        self.average_acc_y = 0.
        self.average_acc_z = 0.

    def listener_callback(self, msg):
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z

    def timer_callback(self):
        if self.count < 1000:
            # compute the average
            self.average_acc_x += self.acc_x
            self.average_acc_y += self.acc_y
            self.average_acc_z += self.acc_z
            print(self.count)
            self.count += 1
        else:
            self.average_acc_x /= 1000
            self.average_acc_y /= 1000
            self.average_acc_z /= 1000
            print(f"Average acceleration: {self.average_acc_x}, {self.average_acc_y}, {self.average_acc_z}")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()