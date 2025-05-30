#ifndef PAMI_PATHS_H
#define PAMI_PATHS_H

#include "Config/DEFINE_PAMI.h"


#define MAX_SPEED 255 // [0, 255]
#define LOW_SPEED 120



#define COLOR_YELLOW 0
#define COLOR_BLUE 1


struct Segment {
  // it means that we turn the servo, and then go at speed during duration_s
  int dir_servo_angle; // name of direction
  float duration_s; // duration of TRACTION servo
  int speed; // in servo unit [-512, 512] // at which speed
  int angle; // at which angle of DIR servo
};

extern Segment path[50];
extern int path_length;

void make_path(int color);

#endif //PAMI_PATHS_H
