#ifndef ACTUATORSTASK_H
#define ACTUATORSTASK_H
#include <cstdint>

extern bool stop_all_actuators_requested;

extern uint8_t LEFT_ARM_0_SERVO_ID;
extern uint8_t LEFT_ARM_1_SERVO_ID;
extern uint8_t LEFT_ARM_2_SERVO_ID;
extern uint8_t LEFT_ARM_3_SERVO_ID;

extern uint8_t RIGHT_ARM_0_SERVO_ID;
extern uint8_t RIGHT_ARM_1_SERVO_ID;
extern uint8_t RIGHT_ARM_2_SERVO_ID;
extern uint8_t RIGHT_ARM_3_SERVO_ID;

// THERMOMETER SERVO
static constexpr int THERMO_SERVO_ID = 7;                     // TODO
static constexpr int THERMO_SERVO_OPEN   = 150;    // ° [0,270] // TODO
static constexpr int THERMO_SERVO_CLOSED = 40;  // ° [0,270] // TODO

void ActuatorsTaskStart();

void raiseThermometerServo();
void lowerThermometerServo();
void initEveryThing();
void handleManualRequests();

#endif //ACTUATORSTASK_H
