#include "Devices/PosStepper.h"
#include "Config/Config.h"
#include "cmsis_os2.h"
#include <Util/SpeedLimits.h>

PosStepper::PosStepper() = default;

PosStepper::PosStepper(const TIM_HandleTypeDef &tim_handle_step,
                       const uint32_t tim_channel_step,
                       GPIO_TypeDef *gpio_port_dir, const uint16_t gpio_pin_dir,
                       int inverse_dir=1)
    : _speed_stepper(tim_handle_step, tim_channel_step, gpio_port_dir,
                     gpio_pin_dir, inverse_dir) {

  _max_speed = STEPPER_MAX_SPEED_DEFAULT;
  _max_accel = STEPPER_MAX_ACCEL_DEFAULT;
  _current_pos = 0;
  _goal_pos = 0;
  _pos_reached = true;
}

void PosStepper::set_max_speed(float speed) { _max_speed = speed; }

void PosStepper::set_max_accel(float accel) { _max_accel = accel; }

void PosStepper::set_goal_async(float goal_pos) {
  if (!_pos_reached) {
    cancel_goal();
  }
  _goal_pos = goal_pos;
  _pos_reached = false;
  osDelay(1); // just in case
}

void PosStepper::set_goal_sync(float goal_pos) {
  set_goal_async(goal_pos);

  while (!_pos_reached) {
    osDelay(10);
  }
}

void PosStepper::set_zero() { _current_pos = 0; }

void PosStepper::cancel_goal() {
  _speed_stepper.set_speed_rps(0);
  _pos_reached = true;
}

bool PosStepper::pos_reached() const { return _pos_reached; }

void PosStepper::spin_once(double dt) {
  if (_pos_reached) {
    return;
  }

  double current_speed = _speed_stepper.get_speed_rps();

  if (!_pos_reached) {
    _current_pos += current_speed * dt;
  }

  float delta_pos = _goal_pos - _current_pos;

  if ((current_speed > 0. && delta_pos <= 0) || (current_speed < 0. && delta_pos >= 0)) {
    cancel_goal();
    _pos_reached = true;
    return;
  }

  const float limited_speed =
      limit_accel(_speed_stepper.get_speed_rps(),
                  _max_speed * (delta_pos > 0 ? 1. : -1.), _max_accel, dt);

  _speed_stepper.set_speed_rps(limited_speed);
}
