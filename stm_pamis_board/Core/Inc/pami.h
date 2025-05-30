#ifndef PAMI_H
#define PAMI_H

#include "Devices/SCServos.h"
#include "Devices/LaserSensor.hpp"
#include "Config/DEFINE_PAMI.h"

#ifdef PAMI_1
#endif
  // TODO
#ifdef PAMI_2
  #define ID_SERVO_DIR 6
#endif
#ifdef PAMI_3
  // TODO
#endif
#ifdef PAMI_SUPERSTAR
  #define ID_SERVO_DIR 10
#endif


#define SENSOR_SENSOR_OBSTACLE_ADDRESS 0x53
#define SENSOR_SENSOR_OBSTACLE_OFFSET 0 // TODO
#define SENSOR_SENSOR_VOID_ADDRESS 0x54
#define SENSOR_SENSOR_VOID_OFFSET 0

#define STOPPING_DISTANCE_MM 20 // TODO
#define STOPPING_DISTANCE_TO_EDGE_MM 20

#define ACTUATOR_POSE1_ANGLE 90
#define ACTUATOR_POSE2_ANGLE 180

extern SCServos servos;

void PAMI_Init();
void PAMI_Main();

#endif //PAMI_H
