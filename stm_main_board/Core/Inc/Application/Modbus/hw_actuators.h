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

  TAKE_2_BOXES,
  BRING_2_BOXES_ON_TOP,
  PUT_2_LAST_BOXES_ON_THE_GROUND,

  PREPARE_TOP_PUSHER,
  GRAB_AND_SORT_2_BOXES_FROM_LIFT,
  PUSH_2_BOXES_OUT,
  OPEN_EXIT_RAMP,

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
  case ActuatorCommand::TAKE_2_BOXES:                   return "TAKE_2_BOXES";
  case ActuatorCommand::BRING_2_BOXES_ON_TOP:           return "BRING_2_BOXES_ON_TOP";
  case ActuatorCommand::PUT_2_LAST_BOXES_ON_THE_GROUND: return "PUT_2_LAST_BOXES_ON_THE_GROUND";
  case ActuatorCommand::PREPARE_TOP_PUSHER:             return "PREPARE_TOP_PUSHER";
  case ActuatorCommand::GRAB_AND_SORT_2_BOXES_FROM_LIFT:return "GRAB_AND_SORT_2_BOXES_FROM_LIFT";
  case ActuatorCommand::PUSH_2_BOXES_OUT:               return "PUSH_2_BOXES_OUT";
  case ActuatorCommand::OPEN_EXIT_RAMP:                 return "OPEN_EXIT_RAMP";
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