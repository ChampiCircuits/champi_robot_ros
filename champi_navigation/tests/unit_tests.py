import unittest
from geometry_msgs.msg import Pose

from math import pi

class TestCalculateAngleToTarget(unittest.TestCase):
    """
    Here we test if the computed difference of angle is correct. Hence we substract robot_pose[2](=yaw_robot)
    """

    def setUp(self) -> None:
        pass


if __name__ == '__main__':
    unittest.main()