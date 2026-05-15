from enum import Enum, auto


class PlannerStatus(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    RUNNING = auto()
    NO_PATH = auto()
    IN_FORBIDDEN_AREA = auto()
    GOAL_REACHED = auto()
    TIMED_OUT = auto()
    CANCELLED = auto()