#ifndef HWACTUATORS_H
#define HWACTUATORS_H

#include <string>


#define ACTUATORS_COUNT 7
/*
  Enum for actuator commands.
*/
enum class ActuatorCommand {
  PUT_BANNER = 1,
  TAKE_PLANKS = 2,
  PUT_PLANKS = 3,
  TAKE_CANS_LEFT = 4,
  TAKE_CANS_RIGHT = 5,
  PUT_CANS_LEFT = 6,
  PUT_CANS_RIGHT = 7
};

/*
 Enum for actuator states.
 - defaults to NOTHING
 - when ROS requests an action, it sets the state to REQUESTED
 - when the action is done, the STM sets the state to DONE
 - when ROS sees that the state is DONE, it sets the state to NOTHING and moves to the next action
*/
enum class ActuatorState {
  NOTHING = 0,
  REQUESTED = 1,
  DONE = 2
};

std::string to_string(ActuatorCommand command)
{
  switch (command)
  {
    case ActuatorCommand::PUT_BANNER:
      return "PUT_BANNER";
    case ActuatorCommand::TAKE_PLANKS:
      return "TAKE_PLANKS";
    case ActuatorCommand::PUT_PLANKS:
      return "PUT_PLANKS";
    case ActuatorCommand::TAKE_CANS_LEFT:
      return "TAKE_CANS_LEFT";
    case ActuatorCommand::TAKE_CANS_RIGHT:
      return "TAKE_CANS_RIGHT";
    case ActuatorCommand::PUT_CANS_LEFT:
      return "PUT_CANS_LEFT";
    case ActuatorCommand::PUT_CANS_RIGHT:
      return "PUT_CANS_RIGHT";
    default:
      return "UNKNOWN_COMMAND";
  }
}

std::string to_string(ActuatorState state) {
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