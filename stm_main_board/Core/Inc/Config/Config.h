#include "main.h"

#ifndef INC_CONFIG_CONFIG_H_
#define INC_CONFIG_CONFIG_H_

// ======================== MISCELLANEOUS ============================
// Values we're lazy to get programmatically :)
#define STEPPER_TIMERS_INPUT_FREQ_HZ 275000000 // Before prescaler

#define SYS_TASK_LOOP_PERIOD_MS 50


// ======================== HOLODRIVE CONFIG ========================

#define CMD_TIMEOUT_MS 1000

#define CONTROL_LOOP_FREQ_HZ 100
#define CONTROL_LOOP_PERIOD_MS (1000 / CONTROL_LOOP_FREQ_HZ)
#define CONTROL_LOOP_PERIOD_S (1.0 / ((float) CONTROL_LOOP_FREQ_HZ))

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

# define WHEEL_B_DIR_GPIO_PORT DIR_REAR_GPIO_Port
# define WHEEL_L_DIR_GPIO_PORT DIR_LEFT_GPIO_Port
#define WHEEL_R_DIR_GPIO_PORT DIR_RIGHT_GPIO_Port

#define WHEEL_B_DIR_GPIO_PIN DIR_REAR_Pin
#define WHEEL_L_DIR_GPIO_PIN DIR_LEFT_Pin
#define WHEEL_R_DIR_GPIO_PIN DIR_RIGHT_Pin


// ======================== POSSTEPPER CONFIG ========================

#define STEPPER_MAX_SPEED_DEFAULT 10.0
#define STEPPER_MAX_ACCEL_DEFAULT 5.0
#define POSSTEPPERS_LOOP_PERIOD_MS 10


// ======================== OTOS CONFIG ==============================

#define OTOS_LOOP_PERIOD_MS 10 // Took this value from an Arduino example

#define OTOS_OFFSET_X 0.0
#define OTOS_OFFSET_Y 0.0
#define OTOS_OFFSET_H (M_PI / 2.0)

// ========================== LOGGING ================================

#define ENABLE_LOG_DEBUG 0 // 0: disable, 1: enable

#endif /* INC_CONFIG_CONFIG_H_ */


// ========================== SCServos CONFIG ========================

#define SCSERVOS_TORQUE_LIMIT 700
