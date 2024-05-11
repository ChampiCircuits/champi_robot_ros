from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
import rclpy


# create a node to pub on /act_status which is a Int64MultiArray with [5,0]

class PubStatusTest(Node):

    def __init__(self):
        super().__init__('pub_status_test')
        self.publisher_ = self.create_publisher(Int64MultiArray, '/act_status', 10)
        timer_period = 1.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('INIT')

    def timer_callback(self):
        msg = Int64MultiArray()
        msg.data = [2, 0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    pub_status_test = PubStatusTest()

    rclpy.spin(pub_status_test)

    pub_status_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
