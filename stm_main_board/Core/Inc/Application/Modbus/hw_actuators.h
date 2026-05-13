#ifndef HWACTUATORS_H
#define HWACTUATORS_H

#include <string>


/*
  Enum for actuator commands.
*/
enum class ActuatorCommand : int
{ // be careful, if thee holo_ order is changed, you must change also holo_teleop_joy_node.py
  RESET_ACTUATORS,
  STOP_ALL_MOTORS,
  ENABLE_ALL_MOTORS,
  GET_READY,

  THERMOMETER_LOWER_SERVO,
  THERMOMETER_RAISE_SERVO,

  LOWER_LEFT_ARM,
  GET_READY_LEFT_ARM,
  LET_GO_ELEMENTS_LEFT_ARM,

  LOWER_RIGHT_ARM,
  GET_READY_RIGHT_ARM,
  LET_GO_ELEMENTS_RIGHT_ARM,

  PUMPS_ON,
  PUMPS_OFF,

  ACTUATORS_COUNT // only to count the number of available commands
};

/*
 Enum for actuator states.
 - defaults to NOTHING
 - when ROS requests an action, it sets the state to REQUESTED
 - when the action is done, the STM sets the state to DONE
 - when ROS sees that the state is DONE, it sets the state to NOTHING and moves to the next action
*/
enum class ActuatorState { NOTHING = 0, REQUESTED = 1, DONE = 2 };

inline const char* to_c_str(const ActuatorCommand command)
{
  switch (command)
  {
  case ActuatorCommand::RESET_ACTUATORS:                return "RESET_ACTUATORS";
  case ActuatorCommand::STOP_ALL_MOTORS:                return "STOP_ALL_MOTORS";
  case ActuatorCommand::ENABLE_ALL_MOTORS:              return "ENABLE_ALL_MOTORS";
  case ActuatorCommand::GET_READY:                      return "GET_READY";
  case ActuatorCommand::THERMOMETER_LOWER_SERVO:        return "THERMOMETER_LOWER_SERVO";
  case ActuatorCommand::THERMOMETER_RAISE_SERVO:        return "THERMOMETER_RAISE_SERVO";
  case ActuatorCommand::LOWER_LEFT_ARM:                 return "LOWER_LEFT_ARM";
  case ActuatorCommand::GET_READY_LEFT_ARM:            return "GET_READY_LEFT_ARM";
  case ActuatorCommand::LET_GO_ELEMENTS_LEFT_ARM:       return "LET_GO_ELEMENTS_LEFT_ARM";
  case ActuatorCommand::LOWER_RIGHT_ARM:                return "LOWER_RIGHT_ARM";
  case ActuatorCommand::GET_READY_RIGHT_ARM:           return "GET_READY_RIGHT_ARM";
  case ActuatorCommand::LET_GO_ELEMENTS_RIGHT_ARM:      return "LET_GO_ELEMENTS_RIGHT_ARM";
  default:
    return "UNKNOWN_COMMAND";
  }
}

inline const char* to_c_str(const ActuatorState state)
{
  switch (state)
  {
    case ActuatorState::NOTHING:  return "NOTHING";
    case ActuatorState::REQUESTED:return "REQUESTED";
    case ActuatorState::DONE:     return "DONE";
    default:
      return "UNKNOWN_STATE";
  }
}



#endif // HWACTUATORS_H