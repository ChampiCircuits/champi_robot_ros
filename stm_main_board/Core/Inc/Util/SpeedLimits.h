#ifndef INC_UTIL_SPEEDLIMITS_H_
#define INC_UTIL_SPEEDLIMITS_H_


// Returns the velocity with limited acceleration and deceleration applied
double limit_accel_decel(double current_speed, double goal_speed, double max_acceleration, double max_deceleration, double dt);

// Returns the velocity with limited acceleration applied
double limit_accel(double current_speed, double goal_speed, double max_acceleration, double dt);

// Returns the velocity with limited deceleration applied
double limit_decel(double current_speed, double goal_speed, double max_deceleration, double dt);


#endif /* INC_UTIL_SPEEDLIMITS_H_ */
