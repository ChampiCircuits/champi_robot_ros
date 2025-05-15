#ifndef HWACTUATORS_H
#define HWACTUATORS_H

#include <string>


#define ACTUATORS_COUNT 10
/*
  Enum for actuator commands.
*/
enum class ActuatorCommand {
  PUT_BANNER = 0,
  TAKE_LOWER_PLANK = 1,
  TAKE_UPPER_PLANK = 2,
  PUT_LOWER_PLANK_LAYER_1 = 3,
  PUT_UPPER_PLANK_LAYER_2 = 4,
  TAKE_CANS_RIGHT = 5,
  TAKE_CANS_LEFT = 6,
  PUT_CANS_RIGHT_LAYER_2 = 7,
  PUT_CANS_LEFT_LAYER_1 = 8,
  RESET_ACTUATORS = 9,
};

/*
 Enum for actuator states.
 - defaults to NOTHING
 - when ROS requests an action, it sets the state to REQUESTED
 - when the action is done, the STM sets the state to DONE
 - when ROS sees that the state is DONE, it sets the state to NOTHING and moves to the next action
*/
enum class ActuatorState { NOTHING = 0, REQUESTED = 1, DONE = 2 };

inline std::string to_string(ActuatorCommand command) {
  switch (command) {
  case ActuatorCommand::PUT_BANNER:
    return "PUT_BANNER";
  case ActuatorCommand::TAKE_LOWER_PLANK:
    return "TAKE_LOWER_PLANK";
  case ActuatorCommand::TAKE_UPPER_PLANK:
    return "TAKE_UPPER_PLANK";
  case ActuatorCommand::PUT_LOWER_PLANK_LAYER_1:
    return "PUT_LOWER_PLANK_LAYER_1";
  case ActuatorCommand::PUT_UPPER_PLANK_LAYER_2:
    return "PUT_UPPER_PLANK_LAYER_2";
  case ActuatorCommand::TAKE_CANS_RIGHT:
    return "TAKE_CANS_RIGHT";
  case ActuatorCommand::TAKE_CANS_LEFT:
    return "TAKE_CANS_LEFT";
  case ActuatorCommand::PUT_CANS_RIGHT_LAYER_2:
    return "PUT_CANS_RIGHT_LAYER_2";
  case ActuatorCommand::PUT_CANS_LEFT_LAYER_1:
    return "PUT_CANS_LEFT_LAYER_1";
  case ActuatorCommand::RESET_ACTUATORS:
    return "RESET_ACTUATORS";
  default:
    return "UNKNOWN_COMMAND";
  }
}

inline std::string to_string(ActuatorState state) {
  switch (state) {
    case ActuatorState::NOTHING:
      return "NOTHING";
    case ActuatorState::REQUESTED:
      return "REQUESTED";
    case ActuatorState::DONE:
      return "DONE";
    default:
      return "UNKNOWN_STATE";
  }
}



#endif // HWACTUATORS_H