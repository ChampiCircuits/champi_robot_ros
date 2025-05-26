#include "pami_paths.h"

#include "Util/logging.h"
#include "Config/DEFINE_PAMI.h"

// a segment is  {dir_servo_angle, duration_s, speed}
// it means that we turn the servo dir to angle, and then go at speed during duration_s
// path are defined for color YELLOW by default
#ifdef PAMI_1

Segment path[] = {
  {DIR_ANGLE_STRAIGHT, 3.0, MAX_SPEED},
  {DIR_ANGLE_RIGHT___, 0.5, MAX_SPEED},
  {DIR_ANGLE_LEFT____, 0.5, MAX_SPEED},
  {DIR_ANGLE_STRAIGHT, 6.0, MAX_SPEED},
};

const int path_length = sizeof(path) / sizeof(path[0]);
#endif

#ifdef PAMI_2

Segment path[] = {
  // TODO
};

const int path_length = sizeof(path) / sizeof(path[0]);
#endif

#ifdef PAMI_3

Segment path[] = {
  // TODO
};

const int path_length = sizeof(path) / sizeof(path[0]);
#endif

#ifdef PAMI_SUPERSTAR
// TODO POUR LA SUPERSTAR, LE DERNIER SEGMENT POUR S'APPROCHER DU VIDE EST JUSTE UNE ROTATION, avancer est géré dans le code
Segment path[] = {
  // TODO
};

const int path_length = sizeof(path) / sizeof(path[0]);
#endif


void check_path_duration() {
  int total_duration = 0;

  for (int i=0; i < path_length; i++) {
    const Segment current_segment = path[i];

    // turn the DIR wheel
    total_duration += 200;
    total_duration += 200; // TODO ajuster

    // set TRACTION velocity
    total_duration += static_cast<int>(current_segment.duration_s * 1000);
  }

  LOG_INFO("init", "Total duration is %dms with %d segments", total_duration, path_length);
#ifdef PAMI_SUPERSTAR
  LOG_INFO("init", "But there is also the last segment till the edge of the scene");
#endif
}


void change_directions_according_to_color() {
  for (int i=0; i < path_length; i++) {
#ifdef COLOR_YELLOW
    #define DIR_ANGLE_STRAIGHT 135
    #define DIR_ANGLE_RIGHT___ (135+90)
    #define DIR_ANGLE_LEFT____ (135-90)
#else
    // vertical symmetry
    #define DIR_ANGLE_STRAIGHT 135
    #define DIR_ANGLE_RIGHT___ (135-90)
    #define DIR_ANGLE_LEFT____ (135+90)
#endif
  }
}