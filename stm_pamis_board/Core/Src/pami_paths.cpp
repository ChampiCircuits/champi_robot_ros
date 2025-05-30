#include "pami_paths.h"

#include "pami.h"
#include "Util/logging.h"
#include "Config/DEFINE_PAMI.h"

#define DIR_STRAIGHT 0
#define DIR_RIGHT___ 1
#define DIR_LEFT____ 2

// a segment is  {dir_servo_angle, duration_s, speed}
// it means that we turn the servo dir to angle, and then go at speed during duration_s
// path are defined for color YELLOW by default
#ifdef PAMI_1


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

Segment path[] = {
  {DIR_STRAIGHT, 3.0, MAX_SPEED},
  {DIR_LEFT____, 2.65, MAX_SPEED},
  {DIR_STRAIGHT, 3.0, low},
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

void change_path_according_to_color()
{
  LOG_INFO("init", "Checking color...");
  // read COLOR_BTN
  int color;
  if (HAL_GPIO_ReadPin(COLOR_BTN_GPIO_Port, COLOR_BTN_Pin) == GPIO_PIN_SET)
  {
    LOG_INFO("init", "Color = YELLOW");
    color = YELLOW;
  } else
  {
    LOG_INFO("init", "Color = BLUE");
    color = BLUE;
  }

  for (int i=0; i<path_length; i++)
  {
    if (color == YELLOW)
    {
      if (path[i].dir_servo_angle ==DIR_RIGHT___)
      {
        path[i].angle = DIR_ANGLE_RIGHT___;
        LOG_INFO("init", "Angle right = %d", path[i].angle);
      }
      else if (path[i].dir_servo_angle == DIR_LEFT____)
      {
        path[i].angle = DIR_ANGLE_LEFT____;
        LOG_INFO("init", "Angle left = %d", path[i].angle);
      }
    }
    else
    {
      if (path[i].dir_servo_angle ==DIR_RIGHT___)
      {
        path[i].angle = DIR_ANGLE_LEFT____; // INVERSED
      }
      else if (path[i].dir_servo_angle ==DIR_LEFT____)
      {
        path[i].angle = DIR_ANGLE_RIGHT___; // INVERSED
      }
    }
    if (path[i].dir_servo_angle == DIR_STRAIGHT)
    {
      path[i].angle = DIR_ANGLE_STRAIGHT;
      LOG_INFO("init", "Angle straight = %d", path[i].angle);
    }
    LOG_INFO("init", "Segment %d: angle = %d, speed = %d", i, path[i].angle, path[i].speed);
  }
}
