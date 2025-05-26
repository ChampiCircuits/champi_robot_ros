#ifndef PAMI_H
#define PAMI_H

#include "Devices/SCServos.h"
#include "Devices/LaserSensor.hpp"
#include "Config/DEFINE_PAMI.h"

#ifdef PAMI_1
  #define ID_SERVO_DIR 18
  #define ID_SERVO_TRACTION 17
#endif
  // TODO
#ifdef PAMI_2
#endif
#ifdef PAMI_3
  // TODO
#endif
#ifdef PAMI_SUPERSTAR
  // TODO
#endif


#define SENSOR_SENSOR_OBSTACLE_ADDRESS 0x53
#define SENSOR_SENSOR_OBSTACLE_OFFSET 0 // TODO
#define SENSOR_SENSOR_VOID_ADDRESS 0x54
#define SENSOR_SENSOR_VOID_OFFSET 0

#define STOPPING_DISTANCE_MM 20 // TODO
#define STOPPING_DISTANCE_TO_EDGE_MM 20

extern SCServos servos;
extern LaserSensor sensor_obstacle;
extern LaserSensor sensor_void;

void PAMI_Init();
void PAMI_Main();

#endif //PAMI_H
