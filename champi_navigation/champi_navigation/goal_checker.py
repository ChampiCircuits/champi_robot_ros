from math import hypot, pi

from champi_libraries_py.data_types.geometry import Pose2D
from champi_interfaces.msg import CtrlGoal

from icecream import ic



def is_goal_reached(goal_pose:Pose2D, current_pose:Pose2D, arrival_speed_0:bool, do_look_at_point:bool, look_at_point:Pose2D, linear_tolerance:float, angular_tolerance:float):

    if arrival_speed_0:


        if not do_look_at_point:  # goal reached -> pose reached
            return is_pose_reached(goal_pose, current_pose, linear_tolerance, angular_tolerance)
        
        # if look_at_point is set, goal reached -> position reached then error angle to look at point = 0

        angle_error = abs(current_pose.get_angle_difference_to_look_at(look_at_point))
        return is_position_reached(goal_pose, current_pose, linear_tolerance) and angle_error < angular_tolerance
    

    # If we reach this, arrival speed is non-zero. In that case, we only check if the position is reached.
    # If we were waiting for the orientation to be good, the robot would be turning around the point at arrival_speed
    # untill the orientation is reached.

    return is_position_reached(goal_pose, current_pose, linear_tolerance)


def is_ctrl_goal_reached(goal:CtrlGoal, current_pose:Pose2D):

    return is_goal_reached(
        Pose2D(pose=goal.pose),
        current_pose,
        goal.end_speed == 0,
        goal.do_look_at_point,
        Pose2D(point=goal.look_at_point),
        goal.linear_tolerance,
        goal.angular_tolerance)
    




def is_pose_reached(goal:Pose2D, current_pose:Pose2D, linear_tolerance:float, angular_tolerance:float):
    """Check if the goal pose is reached within a linear and angular tolerance.

    Args:
        goal (Pose): Goal pose
        current_pose (Pose): Current position
        linear_tolerance (float): Linear tolerance (meters)
        angular_tolerance (float): Angular tolerance (radians)
    """
    
    return is_position_reached(goal, current_pose, linear_tolerance) and is_angle_reached(goal, current_pose, angular_tolerance)


def is_position_reached(goal:Pose2D, current_pose:Pose2D, linear_tolerance:float):
    """Check is the position goal is reached within a linear tolerance.

    Args:
        goal (Pose): Goal pose (only x and y are considered)
        current_pose (Pose): Current position (only x and y are considered)
        linear_tolerance (float): Linear tolerance (meters)
    """

    return hypot(goal.x - current_pose.x, goal.y - current_pose.y) < linear_tolerance


def is_angle_reached(goal:Pose2D, current_pose_robot:Pose2D, angular_tolerance:float):
    """Check is the position goal is reached within an angular tolerance.

    Args:
        goal (Pose): Goal pose (only z is considered)
        current_pose (Pose): Current position (only z is considered)
        angular_tolerance (float): Angular tolerance (radians)
    """

    return get_minimal_angle_difference(current_pose_robot.theta, goal.theta) < angular_tolerance


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