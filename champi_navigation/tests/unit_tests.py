import unittest
from geometry_msgs.msg import Pose
from champi_navigation.path_helper import PathHelper

from math import pi

class TestCalculateAngleToTarget(unittest.TestCase):
    """
    Here we test if the computed difference of angle is correct. Hence we substract robot_pose[2](=yaw_robot)
    """

    def setUp(self) -> None:
        self.path_helper = PathHelper(0,0,0,0)

    def test_1(self):
        robot_pose = [1,1,0]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, -3*pi/4)

    def test_2(self):
        robot_pose = [-1,-1,0]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, pi/4)

    def test_3(self):
        robot_pose = [0,1,0]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, -pi/2)

    def test_4(self):
        robot_pose = [0,-1,0]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, pi/2)

    def test_with_angle_1(self):
        robot_pose = [1,1,pi]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, pi/4)

    def test_with_angle_2(self):
        robot_pose = [1,1,-pi]
        target_pose = Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.

        angle = self.path_helper.calculate_angle_to_target(robot_pose, target_pose) - robot_pose[2]
        
        self.assertAlmostEqual(angle, pi/4)


if __name__ == '__main__':
    unittest.main()