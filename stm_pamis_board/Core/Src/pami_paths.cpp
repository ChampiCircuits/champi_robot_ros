#include "pami_paths.h"

#include "pami.h"
#include "Util/logging.h"
#include "Config/DEFINE_PAMI.h"



// a segment is  {dir_servo_angle, duration_s, speed}
// it means that we turn the servo dir to angle, and then go at speed during duration_s
// path are defined for color YELLOW by default

#ifdef PAMI_2

Segment path_yellow[] = {
  {DIR_ANGLE_LEFT_____CONST, 1.0, MAX_SPEED},
  {DIR_ANGLE_RIGHT____CONST, 1.0, MAX_SPEED},
  {DIR_ANGLE_STRAIGHT_CONST, 3.0, MAX_SPEED},
};

Segment path_blue[] = {
  {DIR_ANGLE_RIGHT____CONST, 1.0, MAX_SPEED},
//  {DIR_STRAIGHT, 2.0, MAX_SPEED},
  {DIR_ANGLE_LEFT_____CONST, 1.0, MAX_SPEED},
  {DIR_ANGLE_STRAIGHT_CONST, 3.0, MAX_SPEED},
};


#endif

#ifdef PAMI_SUPERSTAR

Segment path_blue[] = {
  {DIR_STRAIGHT, 5.0, MAX_SPEED},
  {DIR_LEFT____, 1.0, MAX_SPEED},

};

Segment path_yellow[] = {
  {DIR_STRAIGHT, 5.0, MAX_SPEED},
  {DIR_LEFT____, 1.0, MAX_SPEED},

};




const int path_length = sizeof(path) / sizeof(path[0]);
#endif

Segment path[];
int path_length;

void make_path(int color) {
  if (color == COLOR_BLUE) {
    *path = *path_blue;
    path_length = sizeof(path) / sizeof(path_blue[0]);
  }
  else if (color == COLOR_YELLOW)
  {
    *path = *path_yellow;
    path_length = sizeof(path) / sizeof(path_yellow[0]);
  }
}

