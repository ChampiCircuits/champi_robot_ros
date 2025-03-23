#ifndef INC_DEVICES_POSSTEPPER_H_
#define INC_DEVICES_POSSTEPPER_H_
#include "SpeedStepper.h"

class PosStepper {

public:
  PosStepper(); // default constructor, do not use.

  PosStepper(const TIM_HandleTypeDef &tim_handle_step, uint32_t tim_channel_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir);

  void set_max_speed(float speed);

  void set_max_accel(float accel);

  void set_goal(float goal_pos);

  void set_zero();

  void cancel_goal();

  bool pos_reached() const;

  void spin_once(double dt);

private:
  SpeedStepper _speed_stepper;
  float _max_speed{};
  float _max_accel{};
  float _current_pos{};
  float _goal_pos{};
  bool _pos_reached{};
};




#endif /* INC_DEVICES_POSSTEPPER_H_ */
