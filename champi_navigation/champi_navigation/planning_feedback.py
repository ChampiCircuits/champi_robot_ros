from enum import Enum
from math import isfinite
from math import hypot

from champi_interfaces.action import Navigate


def _byte_result_to_int(value) -> int:
    """Converts enum/int/bytes-like values to an integer in [0, 255]."""
    if isinstance(value, Enum):
        value = value.value

    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        if len(raw) != 1:
            raise ValueError(f'Expected one byte for path_compute_result, got {len(raw)} bytes')
        return raw[0]

    int_value = int(value)
    if not 0 <= int_value <= 255:
        raise ValueError(f'path_compute_result out of range for byte: {int_value}')
    return int_value


def _int_to_ros_byte(value: int) -> bytes:
    return bytes([value])


def get_feedback_msg(result, path, robot_speed):
    feedback = Navigate.Feedback()

    if result is None:
        result = getattr(
            Navigate.Feedback,
            'INITIALIZING',
            getattr(Navigate.Feedback, 'INTITIALIZING')
        )

    result_int = _byte_result_to_int(result)
    success_straight = _byte_result_to_int(Navigate.Feedback.SUCCESS_STRAIGHT)
    success_avoidance = _byte_result_to_int(Navigate.Feedback.SUCCESS_AVOIDANCE)

    if result_int in (success_straight, success_avoidance):
        feedback.eta = get_estimated_eta(path, robot_speed)
    else:
        feedback.eta = -1.

    feedback.path_compute_result = _int_to_ros_byte(result_int)
    
    return feedback


def get_estimated_eta(path, speed):
    if speed is None or not isfinite(speed) or speed <= 0.0:
        return -1.

    distance = 0
    for i in range(1, len(path)):
        d = hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y)
        distance += d
    return distance / speed

