import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

class DiagnosticsSubscriber(Node):

    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def diagnostics_callback(self, msg):
        self.get_logger().info("Received Diagnostics Array:")
        for status in msg.status:
            self.get_logger().info(f"Node: {status.name}")
            self.get_logger().info(f"- Level: {status.level}, Message: {status.message}")
            # for diagnostic in status.values:
            #     self.get_logger().info(f"  - {diagnostic.key}: {diagnostic.value}")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
