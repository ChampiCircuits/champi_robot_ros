import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

from champi_libraries_py.rviz_marker_display.canva import *
import champi_libraries_py.rviz_marker_display.line_properties as line_properties



class TestNode(Node):
    def __init__(self):
        super().__init__('test_markers')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.canva = Canva(self, enable=True)

        

        self.angle=0.


    def timer_callback(self):

        self.canva.clear()
        self.canva.add(Rectangle((0, 0), 1, 1, self.angle))
        self.canva.add(Line((0, 0), (1, 1), line_properties.line_thin))
        self.canva.draw()

        self.angle += 0.1


        



def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()