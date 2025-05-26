#ifndef PAMI_PATHS_H
#define PAMI_PATHS_H


#define DIR_ANGLE_STRAIGHT 0 // later defined in change_path_according_to_color()
#define DIR_ANGLE_RIGHT___ 0
#define DIR_ANGLE_LEFT____ 0

#define MAX_SPEED 511
#define LOW_SPEED 300


struct Segment {
  // it means that we turn the servo, and then go at speed during duration_s
  float dir_servo_angle; // at which angle of DIR servo
  float duration_s; // duration of TRACTION servo
  int speed; // in servo unit [-512, 512] // at which speed
};

extern Segment path[];
extern const int path_length;

void check_path_duration();
void change_directions_according_to_color();

#endif //PAMI_PATHS_H
