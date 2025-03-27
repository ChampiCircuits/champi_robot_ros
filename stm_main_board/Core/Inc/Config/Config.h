#include "main.h"

#ifndef INC_CONFIG_CONFIG_H_
#define INC_CONFIG_CONFIG_H_


// ======================== HOLODRIVE CONFIG ========================

#define CONTROL_LOOP_FREQ_HZ 100 // Hz. Please don't change it, it can cause issues with the timers configurations.
#define CONTROL_LOOP_PERIOD_MS (1000 / CONTROL_LOOP_FREQ_HZ) // ms
#define CONTROL_LOOP_PERIOD_S (1.0 / ((float) CONTROL_LOOP_FREQ_HZ)) // s


/*

 Top view of the robot.
        _______
       /       \
      /         \
WHEEL_L(eft)  WHEEL_R(ight)
      \         /
       \_______/
          |
      WHEEL_B(ack)
*/

# define WHEEL_B_TIMER_NUMBER 13
# define WHEEL_L_TIMER_NUMBER 15
# define WHEEL_R_TIMER_NUMBER 14

# define WHEEL_B_TIMER_CHANNEL TIM_CHANNEL_1
# define WHEEL_L_TIMER_CHANNEL TIM_CHANNEL_2
# define WHEEL_R_TIMER_CHANNEL TIM_CHANNEL_1

# define WHEEL_B_DIR_GPIO_PORT STEP_REAR_GPIO_Port
# define WHEEL_L_DIR_GPIO_PORT STEP_LEFT_GPIO_Port
# define WHEEL_R_DIR_GPIO_PORT STEP_RIGHT_GPIO_Port

# define WHEEL_B_DIR_GPIO_PIN STEP_REAR_Pin
# define WHEEL_L_DIR_GPIO_PIN STEP_LEFT_Pin
# define WHEEL_R_DIR_GPIO_PIN STEP_RIGHT_Pin


// ======================== POSSTEPPER CONFIG ========================

#define STEPPER_MAX_SPEED_DEFAULT 1.0
#define STEPPER_MAX_ACCEL_DEFAULT 1.0


// ======================== OTOS CONFIG ==============================

#define OTOS_ANGULAR_SCALAR 1.07
#define OTOS_LINEAR_SCALAR 0.992
#define OTOS_OFFSET_X 0.0
#define OTOS_OFFSET_Y 0.0
#define OTOS_OFFSET_H (-M_PI / 2.0)



#endif /* INC_CONFIG_CONFIG_H_ */
