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

    def listener_callback(self, msg):
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z

    def timer_callback(self):
        if self.count < 2000:
            self.file.write(f"{self.acc_x} {self.acc_y} {self.acc_z}\n")
            self.count += 1
            print(self.count)
        else:
            self.file.close()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()