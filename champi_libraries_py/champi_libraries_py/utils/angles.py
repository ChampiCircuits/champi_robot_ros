from math import pi, atan2
from geometry_msgs.msg import Pose


def normalize_angle(angle):
    """Normalize an angle between -pi and pi. Has the consequence to find the shortest angle difference."""
    if angle > pi:
        angle -= 2 * pi
    elif angle < -pi:
        angle += 2 * pi
    
    return angle

def get_yaw(Pose):
    """Get the yaw angle from a Pose message."""
    return quat_to_rad(Pose.orientation.z, Pose.orientation.w)


def quat_to_rad(z: float, w: float) -> float:
    """Convert a quaternion (2D components z and w) to a radian angle."""
    return 2 * atan2(z, w)