#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include "hw_actuators.h"

#include <stdint.h>

namespace com_types {

enum class TeamColor : uint8_t
{
  YELLOW,
  BLUE,
  UNKNOWN
};

struct __attribute__((packed)) Vector3 {
  double x;
  double y;
  double theta;
};

struct __attribute__((packed)) HoloDriveConfig {
  double wheel_radius;
  double base_radius;
  // per-wheel angles (rad) and distances (m) - wheel order: 0,1,2
  double wheel_angles[3];
  double wheel_distances[3];
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
  // 4 bools, 1 for each suction cup. numbers are cups from right to left (from the point of view of the robot)
  uint8_t left_suction_cup_0_activation;
  uint8_t left_suction_cup_1_activation;
  uint8_t left_suction_cup_2_activation;
  uint8_t left_suction_cup_3_activation;
  // 4 bools, 1 for each suction cup. numbers are cups from right to left (from the point of view of the robot)
  uint8_t right_suction_cup_0_activation;
  uint8_t right_suction_cup_1_activation;
  uint8_t right_suction_cup_2_activation;
  uint8_t right_suction_cup_3_activation;
};

}

#endif // DATASTRUCTURES_H
