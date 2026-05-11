from enum import Enum
from math import hypot

from champi_interfaces.action import Navigate


def get_feedback_msg(result, path, robot_speed):
    feedback = Navigate.Feedback()

    if result == Navigate.Feedback.SUCCESS_STRAIGHT or result == Navigate.Feedback.SUCCESS_AVOIDANCE:
        feedback.eta = get_estimated_eta(path, robot_speed)
    else:
        feedback.eta = -1.

    feedback.path_compute_result = result
    
    return feedback


def get_estimated_eta(path, speed):
    distance = 0
    for i in range(1, len(path)):
        d = hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y)
        distance += d
    return distance / speed

