#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include "hw_actuators.h"

#include <stdint.h>

namespace com_types {

enum class TeamColor : uint8_t
{
  UNKNOWN,
  YELLOW,
  BLUE
};

struct __attribute__((packed)) Vector3 {
  double x;
  double y;
  double theta;
};

struct __attribute__((packed)) HoloDriveConfig {
  double wheel_radius;
  double base_radius;
  double max_accel_wheel;
  double max_accel_linear;
  double max_decel_linear;
  double max_accel_angular;
  double max_decel_angular;
};

struct __attribute__((packed)) OtosConfig {
  double linear_scalar;
  double angular_scalar;
};

struct __attribute__((packed)) Config {
  bool is_set;
  double cmd_vel_timeout;
  HoloDriveConfig holo_drive_config;
  OtosConfig otos_config;
};

struct __attribute__((packed)) State {
  bool is_read;
  Vector3 measured_vel;
  Vector3 otos_pose;
  bool e_stop_pressed;
  bool tirette_released;
  char safe_check_counter;
  double dummy; // to not be multiple of 32
};

struct __attribute__((packed)) Cmd {
  bool is_read;
  Vector3 cmd_vel;
};

struct __attribute__((packed)) Requests {
  bool request_reset_otos;
  bool request_reset_stm;
  TeamColor team_color;
};

struct __attribute__((packed)) Actuators
{
  // list of actuators commands.
  double dummy; // to not be multiple of 32
  uint8_t requests[static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT)];
  uint8_t left_suction_cups_activation_for_request[static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT)]; // 4 bools, 1 for each suction cup. numbers are cups from left to right
  uint8_t right_suction_cups_activation_for_request[static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT)]; // 4 bools, 1 for each suction cup. numbers are cups from left to right
};

}

#endif // DATASTRUCTURES_H
