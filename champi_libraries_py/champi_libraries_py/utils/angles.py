from math import pi, atan2
from geometry_msgs.msg import Pose


def normalize_angle(angle: float) -> float:
    """Normalize an angle between -pi and pi. Has the consequence to find the shortest angle difference."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    
    return angle

def get_yaw(pose: Pose) -> float:
    """Get the yaw angle from a Pose message."""
    return quat_to_rad(pose.orientation.z, pose.orientation.w)


def quat_to_rad(z: float, w: float) -> float:
    """Convert a quaternion (2D components z and w) to a radian angle."""
    return 2 * atan2(z, w)