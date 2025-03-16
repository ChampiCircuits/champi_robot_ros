#ifndef INC_DEVICES_STEPPERTIMER_H_
#define INC_DEVICES_STEPPERTIMER_H_

#include "stm32h7xx_hal.h"


class StepperTimer {
public:

	StepperTimer(); // default constructor, do not use.

	StepperTimer(TIM_HandleTypeDef tim_handle_step, uint32_t tim_channel_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir);

	void set_speed_step_freq(int hz, int dir);

	void set_speed_rps(float rps);

	virtual ~StepperTimer();

private:

	void PWM_set_high_duration(TIM_TypeDef* timx, int us) const;

	void PWM_set_freq(TIM_TypeDef* timx, int hz) const;

	TIM_HandleTypeDef tim_handle;
	uint32_t tim_channel;

	GPIO_TypeDef *gpio_port_dir;
	uint16_t gpio_pin_dir;

	bool stopped;
	int current_dir;

	uint32_t timer_input_hz;
};


#endif /* INC_DEVICES_STEPPERTIMER_H_ */
