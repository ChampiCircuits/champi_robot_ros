#include "Util/SpeedLimits.h"



// Returns the velocity with limited acceleration and deceleration applied
double limit_accel_decel(double current_speed, double goal_speed, double max_acceleration, double max_deceleration, double dt) {
        // Check if we are at constant speed, accelerating or decelerating
        if (goal_speed == current_speed) {
            // We are at constant speed
            return current_speed;
        } else if ((goal_speed > current_speed && current_speed >= 0) || (goal_speed < current_speed && current_speed <= 0)) {
            // We are accelerating
            return limit_accel(current_speed, goal_speed, max_acceleration, dt);
        } else {
            // We are decelerating
            return limit_decel(current_speed, goal_speed, max_deceleration, dt);
        }
    }


// Returns the velocity with limited acceleration applied
double limit_accel(double current_speed, double goal_speed, double max_acceleration, double dt) {
    // check if goal is smaller or greater than current speed
    if (goal_speed > current_speed) {
        // Compute the speed we can reach in dt
        double max_speed = current_speed + max_acceleration * dt;
        // Check if we can reach the goal speed
        if (max_speed < goal_speed) {
            // We can't reach the goal speed
            return max_speed;
        } else {
            // We can reach the goal speed
            return goal_speed;
        }
    } else {
        // Compute the speed we can reach in dt
        double min_speed = current_speed - max_acceleration * dt;
        // Check if we can reach the goal speed
        if (min_speed > goal_speed) {
            // We can't reach the goal speed
            return min_speed;
        } else {
            // We can reach the goal speed
            return goal_speed;
        }
    }
}

// Returns the velocity with limited deceleration applied
double limit_decel(double current_speed, double goal_speed, double max_deceleration, double dt) {
    // check if goal is smaller or greater than current speed
    if (goal_speed > current_speed) {
        // Compute the speed we can reach in dt
        double max_speed = current_speed + max_deceleration * dt;
        // Check if we can reach the goal speed
        if (max_speed < goal_speed) {
            // We can't reach the goal speed
            return max_speed;
        } else {
            // We can reach the goal speed
            return goal_speed;
        }
    } else {
        // Compute the speed we can reach in dt
        double min_speed = current_speed - max_deceleration * dt;
        // Check if we can reach the goal speed
        if (min_speed > goal_speed) {
            // We can't reach the goal speed
            return min_speed;
        } else {
            // We can reach the goal speed
            return goal_speed;
        }
    }
}
