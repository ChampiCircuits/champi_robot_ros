#include "Application/SysTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"

osThreadId_t SysTaskHandle;
const osThreadAttr_t sysTask_attributes = {
    .name = "sys_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void SysTask(void *argument) {

  uint32_t start = osKernelGetTickCount();

  LOG_INFO("sys", "Starting loop.");

  while (true) {

    // Enable steppers if E-Stop is released
    HAL_GPIO_WritePin(
      ENABLE_STEPPERS_GPIO_Port,ENABLE_STEPPERS_Pin,
      HAL_GPIO_ReadPin(BAU_GPIO_Port, BAU_Pin));

    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);

    if (mod_reg::requests->request_reset_stm) {
      LOG_INFO("sys", "Resetting STM.");
      HAL_NVIC_SystemReset();
    }

    xSemaphoreGive(ModbusH.ModBusSphrHandle);


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
