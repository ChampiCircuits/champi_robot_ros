#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <stdint.h>

void stepperControlInit();
void stepperControlCommandMmS(float left_mm_s, float right_mm_s, uint32_t now_us);
void stepperControlTick(uint32_t now_us);
void stepperControlStop();

#endif // STEPPER_CONTROL_H

