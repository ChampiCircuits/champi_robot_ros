#include "Devices/StepperTimer.h"



StepperTimer::StepperTimer() = default;

StepperTimer::StepperTimer(TIM_HandleTypeDef tim_handle_step, uint32_t tim_channel_step, GPIO_TypeDef *gpio_port_dir, uint16_t gpio_pin_dir) {

	this->tim_handle = tim_handle_step;
	this->gpio_port_dir = gpio_port_dir;
	this->gpio_pin_dir = gpio_pin_dir;
	this->tim_channel = tim_channel_step;
	HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&this->tim_handle, this->tim_channel);

	this->current_dir = 0;
	this->stopped = true;

	this->timer_input_hz = HAL_RCC_GetPCLK1Freq(); // TODO check that it works.
	// NOTE: There is also a function HAL_RCC_GetPCLK2Freq() for some timers. But they usually have the same frequency.
}

void StepperTimer::PWM_set_high_duration(TIM_TypeDef *timx, int us) const {
	timx->CCR1 = us * this->timer_input_hz / ((timx->PSC+1)*1000000); // TODO I'm not sure the calculation is correct!
}

void StepperTimer::PWM_set_freq(TIM_TypeDef *timx, int hz) const {
	const uint32_t arr = this->timer_input_hz / ((timx->PSC+1)*hz);
	timx->CNT = 0;
	timx->ARR = arr;
}

void StepperTimer::set_speed_step_freq(int hz, int dir) {
	if(hz < 15){// todo calculer freq min automatiquement
		if(!stopped) {
			PWM_set_high_duration(this->tim_handle.Instance, 0);
			stopped = true;
		}
		return;
	}
	if(stopped) {
		// stepper stopped, start pwm
		PWM_set_high_duration(this->tim_handle.Instance, 10);
		stopped = false;
	}
	PWM_set_freq(this->tim_handle.Instance, hz);

	if(dir==1) {
		HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_SET);
	}else {
		HAL_GPIO_WritePin(this->gpio_port_dir, this->gpio_pin_dir, GPIO_PIN_RESET);
	}
}

void StepperTimer::set_speed_rps(float rps) {
	// 3200 steps per revolution
	int hz = rps * 3200.0;
	if(hz>=0) {
		this->set_speed_step_freq(hz, 1);
	}
	else {
		this->set_speed_step_freq(-hz, 0);
	}
}



StepperTimer::~StepperTimer() = default;
