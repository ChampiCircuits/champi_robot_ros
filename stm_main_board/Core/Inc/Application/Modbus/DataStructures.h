#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <stdint.h>

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

struct Actuators
{
  // list of actuators commands. The number of actuators is 9.
  uint8_t requests[9];
};

}

#endif // DATASTRUCTURES_H
