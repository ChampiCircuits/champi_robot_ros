from enum import Enum
from math import hypot

from champi_interfaces.action import Navigate


class ComputePathResult(Enum):
    SUCCESS_STRAIGHT = 0
    START_NOT_IN_COSTMAP = 1
    GOAL_NOT_IN_COSTMAP = 2
    GOAL_IN_OCCUPIED_CELL = 3
    NO_PATH_FOUND = 4
    SUCCESS_AVOIDANCE = 5
    INITIALIZING = 6


def get_feedback_msg(result, path, robot_speed):
    feedback = Navigate.Feedback()

    if result == ComputePathResult.SUCCESS_STRAIGHT or result == ComputePathResult.SUCCESS_AVOIDANCE:
        feedback.eta = get_estimated_eta(path, robot_speed)
    else:
        feedback.eta = -1.

    feedback.path_compute_result = result_to_compute_path_status(result)
    
    return feedback


def get_estimated_eta(path, speed):
    """
    Estimate the time to reach the end of the path with a given speed (we don't take into account the acceleration)
    """
    distance = 0
    for i in range(1, len(path)):
        d = hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y)
        distance += d
    return distance / speed


def result_to_compute_path_status(result):
    if result == ComputePathResult.SUCCESS_STRAIGHT:
        return Navigate.Feedback.SUCCESS_STRAIGHT
    elif result == ComputePathResult.SUCCESS_AVOIDANCE:
        return Navigate.Feedback.SUCCESS_AVOIDANCE
    elif result == ComputePathResult.GOAL_NOT_IN_COSTMAP:
        return Navigate.Feedback.GOAL_NOT_IN_COSTMAP
    elif result == ComputePathResult.NO_PATH_FOUND:
        return Navigate.Feedback.NO_PATH_FOUND
    elif result == ComputePathResult.START_NOT_IN_COSTMAP:
        return Navigate.Feedback.START_NOT_IN_COSTMAP
    elif result == ComputePathResult.GOAL_IN_OCCUPIED_CELL:
        return Navigate.Feedback.GOAL_IN_OCCUPIED_CELL
    elif result == ComputePathResult.INITIALIZING:
        return Navigate.Feedback.INTITIALIZING
