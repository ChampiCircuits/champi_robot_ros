#include "Application/HoloDrive/HoloDrive.h"

#include "Config/Config.h"
#include "Util/SpeedLimits.h"

#include "math.h"

#define PI 3.14159265359
#define SQRT_2_OVER_2 0.70710678118
#define SQRT_3_OVER_2 0.86602540378

#define SQRT_3_OVER_3 0.5773502692


Vector3 sub(Vector3 vel1, Vector3 vel2) {
	return {vel1.x - vel2.x, vel1.y - vel2.y, vel1.theta - vel2.theta};
}

void sub(const double* arr1, const double* arr2, double* ret) {
	for(int i=0; i<3; i++) {
		ret[i] = arr1[i] - arr2[i];
	}
}

void abs(const double* arr, double* ret) {
	for(int i=0; i<3; i++) {
		if(arr[i]>=0) {
			ret[i] = arr[i];
		}
		else {
			ret[i] = -arr[i];
		}
	}
}

int get_index_max(const double* arr) {
	if(arr[0] >= arr[1] && arr[0] >= arr[2]) {
		return 0;
	}
	else if(arr[1] >= arr[0] && arr[1] >= arr[2]) {
		return 1;
	}
	else {
		return 2;
	}
}

HoloDrive::HoloDrive(const StepperTimer& stepper0, const StepperTimer& stepper1, const StepperTimer& stepper2) {
	this->steppers[0] = stepper0;
	this->steppers[1] = stepper1;
	this->steppers[2] = stepper2;
	this->current_wheels_speeds_rps[0] = 0;
	this->current_wheels_speeds_rps[1] = 0;
	this->current_wheels_speeds_rps[2] = 0;

    this->has_config = false;

}

HoloDrive::HoloDrive() = default;

void HoloDrive::set_cmd_vel(Vector3 cmd) {
	this->cmd_vel = cmd;
}

void HoloDrive::compute_wheels_speeds(Vector3 cmd, double *ret_speeds_rps) {
    double wheel0_mps = 0.5 * this->cmd_vel.y + SQRT_3_OVER_2 * this->cmd_vel.x + this->config_.base_radius * this->cmd_vel.theta;
    double wheel1_mps = 0.5 * this->cmd_vel.y - SQRT_3_OVER_2 * this->cmd_vel.x + this->config_.base_radius * this->cmd_vel.theta;
    double wheel2_mps = - this->cmd_vel.y + this->config_.base_radius * this->cmd_vel.theta;
    // wheel mps -> wheel rps
	double wheel_circumference = this->config_.wheel_radius * 2.0 * PI;
    ret_speeds_rps[0] = wheel0_mps / wheel_circumference;
    ret_speeds_rps[1] = wheel1_mps / wheel_circumference;
    ret_speeds_rps[2] = wheel2_mps / wheel_circumference;
}

void HoloDrive::write_wheels_speeds(double *speeds_rps) {
	for(int i=0; i<3; i++) {
		this->steppers[i].set_speed_rps(speeds_rps[i]);
		this->current_wheels_speeds_rps[i] = speeds_rps[i];
	}
}

void HoloDrive::spin_once_motors_control() {

    // Convenience variables
    double max_accel = config_.max_accel_wheel / (double)CONTROL_LOOP_FREQ_HZ; // max accel per cycle

	// Get cmd vel limited
	Vector3 cmd_vel_limited = this->compute_limited_speed();

	// compare current_vel and cmd_vel wheels speeds to check the required acceleration to transition directly from current to command
	double cmd_wheels_speeds[3]; // rotations per second
	this->compute_wheels_speeds(cmd_vel_limited, cmd_wheels_speeds);

	if(this->current_wheels_speeds_rps[2] != this->current_wheels_speeds_rps[2]) {
		this->current_wheels_speeds_rps[2]--;
	}

	double desired_accels_wheels[3];
	sub(cmd_wheels_speeds, this->current_wheels_speeds_rps, desired_accels_wheels);

	double abs_desired_accels_wheels[3];
	abs(desired_accels_wheels, abs_desired_accels_wheels);
	if(abs_desired_accels_wheels[0] < max_accel && abs_desired_accels_wheels[1] < max_accel && abs_desired_accels_wheels[2] < max_accel) {
		// acceleration requested is ok, no need to accelerate gradually.

		this->write_wheels_speeds(cmd_wheels_speeds);
	}
	else {
		// Trouver la roue qui pose le + problème. On va alors pouvoir réduire les accélérations des 3 roues de façon
		// de façon proportionelle, de façon que la roue qui pose le + problème ait une accélération égale à MAX_ACCEL_PER_CYCLE
		int i_max = get_index_max(abs_desired_accels_wheels);

		double speed_ratio = max_accel / abs_desired_accels_wheels[i_max]; // speed ratio of each original speed to add

		double new_speeds_cmds[3];
		for(int i=0; i<3; i++) {
			new_speeds_cmds[i] = current_wheels_speeds_rps[i] + speed_ratio * desired_accels_wheels[i];
		}

		// set speed
		this->write_wheels_speeds(new_speeds_cmds);
	}

	// Compute / update current vel (linear / angular)
	update_current_vel(this->current_wheels_speeds_rps);


}

Vector3 HoloDrive::get_current_vel() {
	return this->current_vel;
}

void HoloDrive::update_current_vel(const double *speeds_rps) {
	double wheel_circumference = this->config_.wheel_radius * 2.0 * PI;
	double wheel0_mps = speeds_rps[0] * wheel_circumference;
    double wheel1_mps = speeds_rps[1] * wheel_circumference;
    double wheel2_mps = speeds_rps[2] * wheel_circumference;

	this->current_vel.x = SQRT_3_OVER_3 * (wheel0_mps - wheel1_mps);
	this->current_vel.y = (1./3.) * (wheel0_mps + wheel1_mps) - (2./3.) * wheel2_mps;
	this->current_vel.theta = (1./(3.*config_.base_radius)) * (wheel0_mps + wheel1_mps + wheel2_mps);
}

/**
 *
 * @param max_accel en rotation.s^-2
 * @param wheel_radius en mètres
 * @param base_radius en mètres
 */
void HoloDrive::set_config(HoloDriveConfig config) {

	this->config_ = config;

    this->max_accel_per_cycle = config.max_accel_wheel / (double)CONTROL_LOOP_FREQ_HZ;

    this->has_config = true;
}

Vector3 HoloDrive::compute_limited_speed() {

	Vector3 cmd_vel_limited;

	// Limit linear acceleration xy
	double current_speed = sqrt(pow(current_vel.x, 2) + pow(current_vel.y, 2));
	double goal_speed = sqrt(pow(cmd_vel.x, 2) + pow(cmd_vel.y, 2));
	if(goal_speed > config_.max_speed_linear) {
		goal_speed = config_.max_speed_linear;
	}
	double cmd_vxy_limited = limit_accel_decel(current_speed, goal_speed, config_.max_accel_linear, config_.max_decel_linear, CONTROL_LOOP_PERIOD_S);

	double vel_vect_angle;
	if (goal_speed == 0) {
		// Use angle of the current vel of the robot. This is to avoid the robot to go forward when the goal speed is 0 but the robot is still moving
		vel_vect_angle = atan2(current_vel.y, current_vel.x);
	} else {
		vel_vect_angle = atan2(cmd_vel.y, cmd_vel.x);
	}
	cmd_vel_limited.x = cmd_vxy_limited * cos(vel_vect_angle);
	cmd_vel_limited.y = cmd_vxy_limited * sin(vel_vect_angle);

	// Limit angular acceleration z
	cmd_vel_limited.theta = limit_accel_decel(current_vel.theta, cmd_vel.theta, config_.max_accel_angular, config_.max_decel_angular, CONTROL_LOOP_PERIOD_S);

	return cmd_vel_limited;
}

bool HoloDrive::is_configured() { // TODO remove .
    return this->has_config;
}

HoloDrive::~HoloDrive() = default;

