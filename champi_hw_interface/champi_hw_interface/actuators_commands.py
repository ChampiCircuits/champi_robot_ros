from enum import Enum

ACTUATORS_COUNT = 7

class ActuatorCommand(Enum):
    """Enum for actuator commands."""
    PUT_BANNER = 1
    TAKE_PLANKS = 2
    PUT_PLANKS = 3
    TAKE_CANS_LEFT = 4
    TAKE_CANS_RIGHT = 5
    PUT_CANS_LEFT = 6
    PUT_CANS_RIGHT = 7

class ActuatorState(Enum):
    """Enum for actuator states.
        defaults to NOTHING
        when ros request an action, it sets the state to REQUESTED
        when the action is done, the stm sets the state to DONE
        when ros see that the state is DONE, it sets the state to NOTHING and go to the next action
    """
    NOTHING = 0
    REQUESTED = 1
    DONE = 2

def to_string(command):
    """Convert an actuator command to a string."""
    if command == ActuatorCommand.PUT_BANNER:
        return "PUT_BANNER"
    elif command == ActuatorCommand.TAKE_PLANKS:
        return "TAKE_PLANKS"
    elif command == ActuatorCommand.PUT_PLANKS:
        return "PUT_PLANKS"
    elif command == ActuatorCommand.TAKE_CANS_LEFT:
        return "TAKE_CANS_LEFT"
    elif command == ActuatorCommand.TAKE_CANS_RIGHT:
        return "TAKE_CANS_RIGHT"
    elif command == ActuatorCommand.PUT_CANS_LEFT:
        return "PUT_CANS_LEFT"
    elif command == ActuatorCommand.PUT_CANS_RIGHT:
        return "PUT_CANS_RIGHT"