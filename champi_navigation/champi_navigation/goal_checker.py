from geometry_msgs.msg import Pose
from math import hypot, atan2, pi


def is_pose_reached(goal:Pose, current_position:Pose, linear_tolerance:float, angular_tolerance:float):
    """Check if the goal pose is reached within a linear and angular tolerance.

    Args:
        goal (Pose): Goal pose
        current_position (Pose): Current position
        linear_tolerance (float): Linear tolerance (meters)
        angular_tolerance (float): Angular tolerance (radians)
    """
    
    return is_position_reached(goal, current_position, linear_tolerance) and is_angle_reached(goal, current_position, angular_tolerance)


def is_position_reached(goal:Pose, current_position:Pose, linear_tolerance:float):
    """Check is the position goal is reached within a linear tolerance.

    Args:
        goal (Pose): Goal pose (only x and y are considered)
        current_position (Pose): Current position (only x and y are considered)
        linear_tolerance (float): Linear tolerance (meters)
    """

    return hypot(goal.position.x - current_position.position.x, goal.position.y - current_position.position.y) < linear_tolerance


def is_angle_reached(goal:Pose, current_position:Pose, angular_tolerance:float):
    """Check is the position goal is reached within an angular tolerance.

    Args:
        goal (Pose): Goal pose (only z is considered)
        current_position (Pose): Current position (only z is considered)
        angular_tolerance (float): Angular tolerance (radians)
    """

    angle_robot = 2 * atan2(current_position.orientation.z, current_position.orientation.w)
    angle_goal = 2 * atan2(goal.orientation.z, goal.orientation.w)

    return get_minimal_angle_difference(angle_robot, angle_goal) < angular_tolerance


def get_minimal_angle_difference(angle1, angle2):
    """Get the minimal difference between two angles.

    Args:
        angle1 (float): First angle (radians)
        angle2 (float): Second angle (radians)
    """

    diff = abs(angle1 - angle2)
    if diff > pi:
        diff = 2 * pi - diff
    return diff