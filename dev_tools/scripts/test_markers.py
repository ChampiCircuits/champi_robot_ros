import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose

import champi_libraries_py.data_types.geometry as geo

from champi_libraries_py.marker_helper.canva import *

import time



class TestNode(Node):
    def __init__(self):
        super().__init__('test_markers')

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        Canva(self, enable=True)

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
        pose2.orientation.x = 1.
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

        self.some_poses2D = [geo.Pose2D(pose=pose1),geo.Pose2D(pose=pose2), geo.Pose2D(pose=pose3), geo.Pose2D(0.5, 0.3, 1.3)]

        self.angle=0.


    def timer_callback(self):


        t_start = time.time()

        Canva().clear()

        Canva().add(items.Polyline(self.some_poses, size=presets.LINE_MEDIUM))
        Canva().add(items.OrientedCube(self.some_poses[0], size=(0.1, 0.2, 0.3)))
        Canva().add(items.Sphere((0.5, 1), size=0.4))
        Canva().add(items.Arrows(self.some_poses2D))
        Canva().add(items.Cubes(self.some_poses, color=presets.MAGENTA))
        Canva().add(items.Cylinders(self.some_poses, color=presets.GREEN))
        Canva().add(items.OrientedCubes(self.some_poses, color=presets.GOLD))
        Canva().add(items.Points(self.some_poses, color=presets.BROWN))
        Canva().add(items.Spheres(self.some_poses, color=presets.NAVY), frame_id='odom')

        Canva().draw()

        print('Time elapsed (ms):', (time.time()-t_start)*1000)

        # 5 ms when enabled
        # 0.02 ms when disabled


        



def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()