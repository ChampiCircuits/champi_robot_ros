#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H
#include "hw_actuators.h"

namespace com_types {

struct Vector3 {
  double x;
  double y;
  double theta;
};

struct HoloDriveConfig {
  double wheel_radius;
  double base_radius;
  double max_accel_wheel;
  double max_accel_linear;
  double max_decel_linear;
  double max_accel_angular;
  double max_decel_angular;
};

struct OtosConfig {
  double linear_scalar;
  double angular_scalar;
};

struct Config {
  bool is_set;
  double cmd_vel_timeout;
  HoloDriveConfig holo_drive_config;
  OtosConfig otos_config;
};

struct State {
  bool is_read;
  Vector3 measured_vel;
  Vector3 otos_pose;
};

struct Cmd {
  bool is_read;
  Vector3 cmd_vel;
};

struct Requests {
  bool request_reset_otos;
  bool request_reset_stm;
};

struct ActuatorsRequests
{
  // list of actuators commands. The number of actuators is 7.
  ActuatorState actuators_requests[ACTUATORS_COUNT];
};

struct ActuatorsStates {
  ActuatorState actuators_states[ACTUATORS_COUNT];
};

}

#endif // DATASTRUCTURES_H
