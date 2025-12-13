import math
import enum

CANID_MOTOR_ALIVE = 0x100
CANID_MOTOR_STATUS = 0x101
CANID_MOTOR_LINE = 0x102
CANID_MOTOR_ROTATE = 0x103
CANID_MOTOR_DONE = 0x104
CANID_RASPI_ALIVE = 0x500

MOT_MAX_VEL = 32.0 # rad/s
MOT_TICKS_PER_REV = 1024*14
ODO_WHEEL_PERIMETER = math.pi * 48.0 # in mm
ODO_TICKS_PER_REV = 16384
ODO_WHEEL_SPACING = 250.0-0.5 # in mm (webots seems to doesn't use wheel center for calculations not clue why, hence the minus something to correct it :)

class MotionState(enum.Enum):
    STAY_AT_POSITION = 0,
    LINE = 1,
    ROTATE = 2

def clip(val, min_, max_):
    if val < min_:
        return min_
    if val > max_:
        return max_
    return val