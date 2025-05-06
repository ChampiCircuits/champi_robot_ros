#include "Application/OtosTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds.h"
#include "Config/Config.h"
#include "Devices/QwiicOTOS.h"
#include "Util/logging.h"

#include "i2c.h"

#include "cmsis_os2.h"
#include "semphr.h"

osThreadId_t OtosTaskHandle;
const osThreadAttr_t otosTask_attributes = {
    .name = "otos_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void OtosTask(void *argument) {
  QwiicOTOS myOtos(&hi2c4, 0x17);
  led_otos::setRed();
  led_otos::SetBrightness(20);

  while (!myOtos.isConnected()) {
    LOG_WARN("otos", "Connecting failed. Retrying...");
    led_otos::setOrange();
    osDelay(1000);
    led_otos::clear(); // TODO does not work ? setbrightness instead ?
    osDelay(1000);
  }

  if (!myOtos.selfTest()) {
    // ERROR WITH OTOS
    LOG_WARN("otos", "Self-test failed.");
    // osDelay(1000000000); // TODO reset STM ?
    while (1) {
      led_otos::setRed();
      osDelay(1000);
      led_otos::clear(); // TODO does not work ? setbrightness instead ?
      osDelay(1000);
    }
  }

  led_otos::setRed();

  // We wait for the config to be set by the master
  while (!mod_reg::config->is_set) {
    osDelay(100);
    LOG_WARN_THROTTLE("otos", 100, "Waiting for config...");
  }

  myOtos.setAngularScalar(mod_reg::config->otos_config.angular_scalar);
  myOtos.setLinearScalar(mod_reg::config->otos_config.linear_scalar);

  myOtos.setOffset({OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H});

  myOtos.calibrateImu();
  osDelay(10);

  myOtos.resetTracking();
  osDelay(10);

  uint32_t start = osKernelGetTickCount();

  LOG_INFO("otos", "Starting loop.");
  led_otos::setGreen();

  while (true) {

    if (mod_reg::requests->request_reset_otos) {
      LOG_INFO("otos", "Resetting OTOS.");
      mod_reg::requests->request_reset_otos = false;
      myOtos.calibrateImu();
      // myOtos.resetTracking(); // We don't reset because jumps are bad for UKF. Still it's a good time to calibrate the IMU
      osDelay(10);
    }

    Pose2D otosPose = myOtos.getPosition();


    LOG_DEBUG_THROTTLE("otos", 100, "reading: \t%f, \t%f, \t%f", otosPose.x,
                       otosPose.y, otosPose.h);

    xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
    mod_reg::state->otos_pose = {otosPose.x, otosPose.y, otosPose.h};
    xSemaphoreGive(ModbusH.ModBusSphrHandle);

    int time_to_wait = OTOS_LOOP_PERIOD_MS - (int)osKernelGetTickCount() + (int)start;
    if (time_to_wait > 0) {
      // avoid negative delay (if calibration takes longer)
      osDelay((uint32_t)time_to_wait);
    }
    start = osKernelGetTickCount();
  }
}

void OtosTaskStart() {
  OtosTaskHandle = osThreadNew(OtosTask, NULL, &otosTask_attributes);
}
