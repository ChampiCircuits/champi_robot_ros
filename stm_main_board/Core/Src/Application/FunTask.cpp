#include "Application/FunTask.h"

#include "usart.h"
#include "math.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"


osThreadId_t FunTaskHandle;
const osThreadAttr_t funTask_attributes = {
  .name = "fun_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

void FunTask(void *argument) {


  LOG_INFO("fun", "Starting loop.");

  while (true) {
    // osDelay(2000);
  }
}

void FunTaskStart() {
  FunTaskHandle = osThreadNew(FunTask, NULL, &funTask_attributes);
}
