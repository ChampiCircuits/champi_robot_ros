import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose

from champi_libraries_py.rviz_marker_display.canva import *



class TestNode(Node):
    def __init__(self):
        super().__init__('test_markers')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.canva = Canva(self, enable=True)


        pose1 = Pose()
        pose1.position.x = 0.5
        pose1.position.y = 0.5
        pose1.position.z = 0.5
        pose1.orientation.x = 0.1
        pose1.orientation.y = 0.5
        pose1.orientation.z = 0.5
        pose1.orientation.w = 0.5

        pose2 = Pose()
        pose2.position.x = -0.5
        pose2.position.y = -0.5
        pose2.position.z = -0.5
        pose2.orientation.x = -0.5
        pose2.orientation.y = -0.5
        pose2.orientation.z = -0.5
        pose2.orientation.w = -0.5

        pose3 = Pose()
        pose3.position.x = 0.5
        pose3.position.y = -0.5
        pose3.position.z = 0.5
        pose3.orientation.x = 0.5
        pose3.orientation.y = -0.5
        pose3.orientation.z = 0.5
        pose3.orientation.w = 0.5

        self.some_poses = [pose1, pose2, pose3]

        self.angle=0.


    def timer_callback(self):

        self.canva.clear()
        self.canva.add(Line((-1, 0), (1, 0), line_width=MEDIUM, color=colors.RED))
        self.canva.add(Polyline(self.some_poses, line_width=MEDIUM, color=colors.GREEN))
        self.canva.add(Poses(self.some_poses, size= 0.2, color=colors.GREEN, type=Poses.Types.CUBES))

        self.canva.add(Cube(self.some_poses[0], size=(0.1, 0.2, 0.3)))
        self.canva.add(Sphere((0.5, 1), radius=0.4, color=colors.BLUE))
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