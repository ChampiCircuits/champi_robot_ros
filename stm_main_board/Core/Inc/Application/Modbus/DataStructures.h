#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

namespace com_types {
struct Vector3 {
  double x;
  double y;
  double theta;
};

struct HoloDriveConfig {
  double wheel_radius;
  double base_radius;
  double max_speed_linear;
  double max_accel_wheel;
  double max_accel_linear;
  double max_decel_linear;
  double max_accel_angular;
  double max_decel_angular;
};

struct Config {
  bool is_set;
  double cmd_vel_timeout;
  HoloDriveConfig holo_drive_config;
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

}

#endif // DATASTRUCTURES_H
