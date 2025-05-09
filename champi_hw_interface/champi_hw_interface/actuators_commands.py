from enum import Enum

ACTUATORS_COUNT = 9

class ActuatorCommand(Enum):
    """Enum for actuator commands."""
    PUT_BANNER = 0
    TAKE_LOWER_PLANK = 1
    TAKE_UPPER_PLANK = 2
    PUT_LOWER_PLANK_LAYER_1 = 3
    PUT_UPPER_PLANK_LAYER_2 = 4
    TAKE_CANS_FRONT = 5
    TAKE_CANS_SIDE = 6
    PUT_CANS_FRONT_LAYER_1 = 7
    PUT_CANS_SIDE_LAYER_2 = 8
    RESET_ACTUATORS = 9

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
    elif command == ActuatorCommand.TAKE_LOWER_PLANK:
        return "TAKE_LOWER_PLANK"
    elif command == ActuatorCommand.TAKE_UPPER_PLANK:
        return "TAKE_UPPER_PLANK"
    elif command == ActuatorCommand.PUT_LOWER_PLANK_LAYER_1:
        return "PUT_LOWER_PLANK_LAYER_1"
    elif command == ActuatorCommand.PUT_UPPER_PLANK_LAYER_2:
        return "PUT_UPPER_PLANK_LAYER_2"
    elif command == ActuatorCommand.TAKE_CANS_FRONT:
        return "TAKE_CANS_FRONT"
    elif command == ActuatorCommand.TAKE_CANS_SIDE:
        return "TAKE_CANS_SIDE"
    elif command == ActuatorCommand.PUT_CANS_FRONT_LAYER_1:
        return "PUT_CANS_FRONT_LAYER_1"
    elif command == ActuatorCommand.PUT_CANS_SIDE_LAYER_2:
        return "PUT_CANS_SIDE_LAYER_2"
    elif command == ActuatorCommand.RESET_ACTUATORS:
        return "RESET_ACTUATORS"
    else:
        return "UNKNOWN"
