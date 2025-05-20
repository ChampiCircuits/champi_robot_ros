#include "Application/SysTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/ActuatorsTask.h"
#include "Application/SCServosApp.h"
#include "Application/OtosTask.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"

#include <Application/Leds/Leds.h>

osThreadId_t SysTaskHandle;
const osThreadAttr_t sysTask_attributes = {
    .name = "sys_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void SysTask(void *argument) {

  LOG_INFO("sys", "Starting loop.");

  uint32_t start = osKernelGetTickCount();

  bool e_stop_pressed_prev = HAL_GPIO_ReadPin(BAU_GPIO_Port, BAU_Pin);
  stop_all_actuators_requested = false;

  while (true) {

    bool tirette_released = HAL_GPIO_ReadPin(TIRETTE_GPIO_Port, TIRETTE_Pin);
    bool e_stop_pressed = HAL_GPIO_ReadPin(BAU_GPIO_Port, BAU_Pin);
    bool e_stop_just_released = e_stop_pressed_prev && !e_stop_pressed;
    e_stop_pressed_prev = e_stop_pressed;

    // ===================== Send status to PC =================================
    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    mod_reg::state->tirette_released = tirette_released;
    mod_reg::state->e_stop_pressed = e_stop_pressed;
    xSemaphoreGive(ModbusH.ModBusSphrHandle);



    bool enableMotors = !(!e_stop_pressed && !stop_all_actuators_requested);

    // ================== Enable / Disable steppers ============================
    HAL_GPIO_WritePin(ENABLE_STEPPERS_GPIO_Port,ENABLE_STEPPERS_Pin, (GPIO_PinState) enableMotors);

    // =================== Handle reset STM requests ===========================
    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    if (mod_reg::requests->request_reset_stm) {
      LOG_INFO("sys", "Resetting STM.");
      HAL_NVIC_SystemReset();
    }
    xSemaphoreGive(ModbusH.ModBusSphrHandle);

    // ================== Enable / Disable servos ==============================
    if (devices::scs_servos::init_successful) {
      devices::scs_servos::set_enable(!enableMotors);
    }

    // ===================== Request otos calibration ==========================
    if (e_stop_just_released) {
      LOG_INFO("sys", "Request Otos calibration");
      otos_calibrate_requested = true;
    }

    // ====================== SysTask period management ========================
    int time_to_wait = SYS_TASK_LOOP_PERIOD_MS - (int)osKernelGetTickCount() + (int)start;
    if (time_to_wait > 0) {
      // avoid negative delay
      osDelay((uint32_t)time_to_wait);
    }
    start = osKernelGetTickCount();
  }
}

void SysTaskStart() {
  SysTaskHandle = osThreadNew(SysTask, NULL, &sysTask_attributes);
}
