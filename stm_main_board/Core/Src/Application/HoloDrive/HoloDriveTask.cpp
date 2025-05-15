#include "Application/HoloDrive/HoloDriveTask.h"
#include <Devices/SpeedStepper.h>

#include "Application/HoloDrive/HoloDrive.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds/Leds.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"
#include "tim.h"

// Macro to convert value from # define WHEEL_B_TIMER 3 to htim3 (or any value
// from 1 to 16)
#define CONCAT(a, b) a##b
#define CONCAT2(a, b) CONCAT(a, b)

#define WHEEL_B_TIMER_HANDLE CONCAT2(htim, WHEEL_B_TIMER_NUMBER)
#define WHEEL_L_TIMER_HANDLE CONCAT2(htim, WHEEL_L_TIMER_NUMBER)
#define WHEEL_R_TIMER_HANDLE CONCAT2(htim, WHEEL_R_TIMER_NUMBER)

osThreadId_t holoDriveTaskHandle;
const osThreadAttr_t holoDriveTask_attributes = {
    .name = "holo_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void HoloDriveTask(void *argument) {
  led_holo::setRed();

  HAL_GPIO_WritePin(ENABLE_STEPPERS_GPIO_Port,ENABLE_STEPPERS_Pin, GPIO_PIN_SET);

  SpeedStepper stepper_back(WHEEL_B_TIMER_HANDLE, WHEEL_B_TIMER_CHANNEL,
                            WHEEL_B_DIR_GPIO_PORT, WHEEL_B_DIR_GPIO_PIN);
  SpeedStepper stepper_left(WHEEL_L_TIMER_HANDLE, WHEEL_L_TIMER_CHANNEL,
                            WHEEL_L_DIR_GPIO_PORT, WHEEL_L_DIR_GPIO_PIN);
  SpeedStepper stepper_right(WHEEL_R_TIMER_HANDLE, WHEEL_R_TIMER_CHANNEL,
                             WHEEL_R_DIR_GPIO_PORT, WHEEL_R_DIR_GPIO_PIN);

  HoloDrive holoDrive(stepper_left, stepper_right, stepper_back);

  led_holo::setBlue();

  // We wait for the config to be set by the master
  while (!mod_reg::config->is_set) {
    LOG_WARN_THROTTLE("holo", 100, "Waiting for config...");
    osDelay(100);
  }

  holoDrive.set_config(mod_reg::config->holo_drive_config);

  LOG_INFO("holo", "Config received. Starting loop.");

  uint32_t start = osKernelGetTickCount();
  uint32_t t_last_read = start;

  led_holo::setGreen();

  while (true) {

    // if cmd vel is read for more than 1 second, stop the motors
    if (mod_reg::cmd->is_read) {
      if (osKernelGetTickCount() - t_last_read > CMD_TIMEOUT_MS) {
        holoDrive.set_cmd_vel(Vector3{0, 0, 0});
        LOG_WARN_THROTTLE("holo", 10, "cmd vel timeout");
        led_holo::setOrange();
      }
      else {
        led_holo::setGreen();
      }
    }
    else {
      xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
      Vector3 cmd = mod_reg::cmd->cmd_vel;
      holoDrive.set_config(mod_reg::config->holo_drive_config); // Update config, Not ideal to do it all the time but it works :)
      holoDrive.set_cmd_vel(cmd);
      mod_reg::cmd->is_read = true;
      xSemaphoreGive(ModbusH.ModBusSphrHandle);
      t_last_read = osKernelGetTickCount();
    }

    holoDrive.spin_once_motors_control();

    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    mod_reg::state->measured_vel = holoDrive.get_current_vel();
    xSemaphoreGive(ModbusH.ModBusSphrHandle);

    // https://community.st.com/t5/stm32cubeide-mcus/freertos-cmsis-v2-osdelayuntil-go-to-hardfaulhandler-only-with/td-p/263884
    // osDelayUntil(start + CONTROL_LOOP_PERIOD_MS);
    uint32_t now = osKernelGetTickCount();
    osDelay(CONTROL_LOOP_PERIOD_MS - now + start);
    start = osKernelGetTickCount();
  }
}

void HoloDriveTaskStart() {
  holoDriveTaskHandle =
      osThreadNew(HoloDriveTask, NULL, &holoDriveTask_attributes);
}
