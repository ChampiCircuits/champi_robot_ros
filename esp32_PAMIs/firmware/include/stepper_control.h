#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

void stepperControlInit();
void stepperControlSetProfile(float speed_mm_s, float accel_mm_s2, float decel_mm_s2);
void stepperControlDrive(float distance_mm);
void stepperControlTurn(float angle_rad);
void stepperControlDecelerateStop();
void stepperControlDisable();
void stepperControlResume();
bool stepperControlIsRunning();
float stepperControlTraveledMm();
void stepperControlRunAtSpeed(float left_mm_s, float right_mm_s);

#endif // STEPPER_CONTROL_H

