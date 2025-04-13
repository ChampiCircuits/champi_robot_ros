#include "Application/OtosTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Devices/QwiicOTOS.h"
#include "Util/logging.h"

#include "i2c.h"

#include "cmsis_os2.h"
#include "semphr.h"

osThreadId_t OtosTaskHandle;
const osThreadAttr_t otosTask_attributes = {
    .name = "holo_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void OtosTask(void *argument) {
  QwiicOTOS myOtos(&hi2c4, 0x17);

  while (!myOtos.isConnected()) {
    LOG_WARN("otos", "Connecting failed. Retrying...");
    osDelay(1000);
  }

  if (!myOtos.selfTest()) {
    // ERROR WITH OTOS
    LOG_WARN("otos", "Self-test failed.");
    osDelay(1000000000); // TODO reset STM ?
  }

  myOtos.setAngularScalar(OTOS_ANGULAR_SCALAR);
  myOtos.setLinearScalar(OTOS_LINEAR_SCALAR);

  myOtos.setOffset({OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H});

  myOtos.calibrateImu();
  osDelay(10);

  myOtos.resetTracking();
  osDelay(10);

  uint32_t start = osKernelGetTickCount();

  LOG_INFO("otos", "Starting loop.");

  while (true) {
    Pose2D otosPose = myOtos.getPosition();

    LOG_DEBUG_THROTTLE("otos", 100, "reading: \t%f, \t%f, \t%f", otosPose.x,
                       otosPose.y, otosPose.h);

    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    mod_reg::state->otos_pose = {otosPose.x, otosPose.y,
                                 otosPose.h}; // TODO check units
    xSemaphoreGive(ModbusH.ModBusSphrHandle);

    uint32_t now = osKernelGetTickCount();
    osDelay(OTOS_LOOP_PERIOD_MS - now + start);
    start = osKernelGetTickCount();
  }
}

void OtosTaskStart() {
  OtosTaskHandle = osThreadNew(OtosTask, NULL, &otosTask_attributes);
}