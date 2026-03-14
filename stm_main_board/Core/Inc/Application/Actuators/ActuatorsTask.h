#ifndef ACTUATORSTASK_H
#define ACTUATORSTASK_H

extern bool stop_all_actuators_requested;

// THERMOMETER SERVO
static constexpr int THERMO_SERVO_ID = 1;                     // TODO
static constexpr int THERMO_SERVO_OPEN   = 0;    // ° [0,270] // TODO
static constexpr int THERMO_SERVO_CLOSED = 90;  // ° [0,270] // TODO

void ActuatorsTaskStart();

void raiseThermometerServo();
void lowerThermometerServo();
void initEveryThing();
void handleManualRequests();

#endif //ACTUATORSTASK_H
