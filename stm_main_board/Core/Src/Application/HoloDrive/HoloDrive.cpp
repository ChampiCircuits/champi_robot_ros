#include "Application/HoloDrive/HoloDrive.h"

#include "logging.h"
#include "Config/Config.h"
#include "Util/SpeedLimits.h"

#include "math.h"

#define PI 3.14159265359
#define SQRT_2_OVER_2 0.70710678118
#define SQRT_3_OVER_2 0.86602540378
#define SQRT_3_OVER_3 0.5773502692

// Hardcoded wheel configuration
// wheel order: 0=left, 1=right, 2=back

constexpr double WHEEL_ANGLES[3] = {60./180.*PI, 300./180.*PI, 180./180.*PI}; // angle between each wheel in radians
constexpr double WHEEL_DISTANCES[3] = {0.13, 0.13, 0.13}; // distance wheel to robot center in m

Vector3 sub(Vector3 vel1, Vector3 vel2) {
  return {vel1.x - vel2.x, vel1.y - vel2.y, vel1.theta - vel2.theta};
}

void sub(const double *arr1, const double *arr2, double *ret) {
  for (int i = 0; i < 3; i++) {
    ret[i] = arr1[i] - arr2[i];
  }
}

void abs(const double *arr, double *ret) {
  for (int i = 0; i < 3; i++) {
    if (arr[i] >= 0) {
      ret[i] = arr[i];
    } else {
      ret[i] = -arr[i];
    }
  }
}

int get_index_max(const double *arr) {
  if (arr[0] >= arr[1] && arr[0] >= arr[2]) {
    return 0;
  } else if (arr[1] >= arr[0] && arr[1] >= arr[2]) {
    return 1;
  } else {
    return 2;
  }
}

HoloDrive::HoloDrive(const SpeedStepper &stepper_left,
                     const SpeedStepper &stepper_right,
                     const SpeedStepper &stepper_back) {
  this->steppers[0] = stepper_left;
  this->steppers[1] = stepper_right;
  this->steppers[2] = stepper_back;
  this->current_wheels_speeds_rps[0] = 0;
  this->current_wheels_speeds_rps[1] = 0;
  this->current_wheels_speeds_rps[2] = 0;

  this->current_vel = {0, 0, 0};
}

HoloDrive::HoloDrive() = default;

void HoloDrive::set_cmd_vel(Vector3 cmd) { this->cmd_vel = cmd; }

void HoloDrive::compute_wheels_speeds(Vector3 cmd_vel, double *ret_speeds_rps) const 
{
  // wheel order: 0=left, 1=right, 2=back

  double cos0 = cos(WHEEL_ANGLES[0]);
  double sin0 = sin(WHEEL_ANGLES[0]);
  double cos1 = cos(WHEEL_ANGLES[1]);
  double sin1 = sin(WHEEL_ANGLES[1]);
  double cos2 = cos(WHEEL_ANGLES[2]);
  double sin2 = sin(WHEEL_ANGLES[2]);

  double wheel0_mps = sin0 * cmd_vel.x - cos0 * cmd_vel.y - WHEEL_DISTANCES[0] * cmd_vel.theta;
  double wheel1_mps = sin1 * cmd_vel.x - cos1 * cmd_vel.y - WHEEL_DISTANCES[1] * cmd_vel.theta;
  double wheel2_mps = sin2 * cmd_vel.x - cos2 * cmd_vel.y - WHEEL_DISTANCES[2] * cmd_vel.theta;

  // wheel mps -> wheel rps
  double wheel_circumference = this->config_.wheel_radius * 2.0 * PI;
  ret_speeds_rps[0] = wheel0_mps / wheel_circumference;
  ret_speeds_rps[1] = wheel1_mps / wheel_circumference;
  ret_speeds_rps[2] = wheel2_mps / wheel_circumference;
}

void HoloDrive::write_wheels_speeds(double *speeds_rps) {
  for (int i = 0; i < 3; i++) {
    this->steppers[i].set_speed_rps(speeds_rps[i]);
    this->current_wheels_speeds_rps[i] = speeds_rps[i];
  }
}

void HoloDrive::spin_once_motors_control() {

  // Convenience variables
  /*
 * max_accel_per_cycle, en rotation par seconde par cycle, est la vitesse
 * maximale autorisée ajoutable à la vitesse actuelle d'une roue à chaque
 * cycle. Soit : Combien peut-on ajouter de vitesse à une roue à chaque cycle
 * de contrôle ?
 */
  double max_accel = config_.max_accel_wheel /
                     (double)CONTROL_LOOP_FREQ_HZ; // max accel per cycle

  // Get cmd vel limited
  Vector3 cmd_vel_limited{};
  cmd_vel_limited.x = limit_accel_decel(
      this->current_vel.x, this->cmd_vel.x, config_.max_accel_linear,
      config_.max_decel_linear, CONTROL_LOOP_PERIOD_S);
  cmd_vel_limited.y = limit_accel_decel(
      this->current_vel.y, this->cmd_vel.y, config_.max_accel_linear,
      config_.max_decel_linear, CONTROL_LOOP_PERIOD_S);
  cmd_vel_limited.theta = limit_accel_decel(
      this->current_vel.theta, this->cmd_vel.theta, config_.max_accel_angular,
      config_.max_decel_angular, CONTROL_LOOP_PERIOD_S);

  // compare current_vel and cmd_vel wheels speeds to check the required
  // acceleration to transition directly from current to command
  double cmd_wheels_speeds[3]; // rotations per second
  this->compute_wheels_speeds(cmd_vel_limited, cmd_wheels_speeds);

  double desired_accels_wheels[3];
  sub(cmd_wheels_speeds, this->current_wheels_speeds_rps,
      desired_accels_wheels);

  double abs_desired_accels_wheels[3];
  abs(desired_accels_wheels, abs_desired_accels_wheels);
  if (abs_desired_accels_wheels[0] < max_accel &&
      abs_desired_accels_wheels[1] < max_accel &&
      abs_desired_accels_wheels[2] < max_accel) {
    // acceleration requested is ok, no need to accelerate gradually.

    this->write_wheels_speeds(cmd_wheels_speeds);
  } else {
    // Trouver la roue qui pose le + problème. On va alors pouvoir réduire les
    // accélérations des 3 roues de façon de façon proportionelle, de façon que
    // la roue qui pose le + problème ait une accélération égale à
    // MAX_ACCEL_PER_CYCLE
    int i_max = get_index_max(abs_desired_accels_wheels);

    double speed_ratio =
        max_accel / abs_desired_accels_wheels[i_max]; // speed ratio of each
                                                      // original speed to add

    double new_speeds_cmds[3];
    for (int i = 0; i < 3; i++) {
      new_speeds_cmds[i] =
          current_wheels_speeds_rps[i] + speed_ratio * desired_accels_wheels[i];
    }

    // set speed
    this->write_wheels_speeds(new_speeds_cmds);
  }

  // Compute / update current vel (linear / angular)
  update_current_vel(this->current_wheels_speeds_rps);
}

Vector3 HoloDrive::get_current_vel() { return this->current_vel; }

void HoloDrive::update_current_vel(const double *speeds_rps) {
  // Inverse kinematics for general wheel positions:
  // A * [vx; vy; omega] = b  where
  // A(i,:) = [cos(angle_i), sin(angle_i), -dist_i]
  double wheel_circumference = this->config_.wheel_radius * 2.0 * PI;
  double b0 = speeds_rps[0] * wheel_circumference;
  double b1 = speeds_rps[1] * wheel_circumference;
  double b2 = speeds_rps[2] * wheel_circumference;

  double a00 = sin(WHEEL_ANGLES[0]);
  double a01 = -cos(WHEEL_ANGLES[0]);
  double a02 = -WHEEL_DISTANCES[0];

  double a10 = sin(WHEEL_ANGLES[1]);
  double a11 = -cos(WHEEL_ANGLES[1]);
  double a12 = -WHEEL_DISTANCES[1];

  double a20 = sin(WHEEL_ANGLES[2]);
  double a21 = -cos(WHEEL_ANGLES[2]);
  double a22 = -WHEEL_DISTANCES[2];

  // Determinant of A
  double detA = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
                a02 * (a10 * a21 - a11 * a20);

  if (fabs(detA) < 1e-9) {
    // Degenerate configuration; avoid division by zero
    this->current_vel.x = 0;
    this->current_vel.y = 0;
    this->current_vel.theta = 0;
    return;
  }

  // Cramer's rule
  double detX = b0 * (a11 * a22 - a12 * a21) - a01 * (b1 * a22 - a12 * b2) +
                a02 * (b1 * a21 - a11 * b2);
  double detY = a00 * (b1 * a22 - a12 * b2) - b0 * (a10 * a22 - a12 * a20) +
                a02 * (a10 * b2 - b1 * a20);
  double detW = a00 * (a11 * b2 - b1 * a21) - a01 * (a10 * b2 - b1 * a20) +
                b0 * (a10 * a21 - a11 * a20);

  this->current_vel.x = detX / detA;
  this->current_vel.y = detY / detA;
  this->current_vel.theta = detW / detA;
}

void HoloDrive::set_config(HoloDriveConfig config) {
  this->config_ = config;
}

HoloDrive::~HoloDrive() = default;

