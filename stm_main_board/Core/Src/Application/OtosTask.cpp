#include "Application/OtosTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds/Leds.h"
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

bool otos_calibrate_requested;

void OtosTask(void *argument) {

  led_otos::setRed();

  osDelay(500); // Let the Otos get powered up.

  QwiicOTOS myOtos(&hi2c4, 0x17);

  while (!myOtos.isConnected()) {
    LOG_WARN("otos", "Connecting failed. Retrying");

    osDelay(1000);
  }

  led_otos::setOrange();

  if (!myOtos.selfTest()) {
    // ERROR WITH OTOS
    LOG_WARN("otos", "Self-test failed. Resetting");
    HAL_NVIC_SystemReset(); // Reset STM
  }

  led_otos::setBlue();

  // We wait for the config to be set by the master
  while (!mod_reg::config->is_set) {
    LOG_WARN_THROTTLE("otos", 100, "Waiting for config...");
    osDelay(100);
  }

  led_otos::setOrange();

  myOtos.setAngularScalar(mod_reg::config->otos_config.angular_scalar);
  myOtos.setLinearScalar(mod_reg::config->otos_config.linear_scalar);

  myOtos.setOffset({OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H});

  myOtos.calibrateImu();
  osDelay(50);

  myOtos.resetTracking();
  osDelay(50);

  uint32_t start = osKernelGetTickCount();

  LOG_INFO("otos", "Starting loop.");
  led_otos::setGreen();

  while (true) {

    if (otos_calibrate_requested) {
      led_otos::setOrange();
      LOG_INFO("otos", "Resetting OTOS.");
      otos_calibrate_requested = false;
      osDelay(2000); // To avoid vibrations following pressing the button.
      myOtos.calibrateImu();
      osDelay(500);
      led_otos::setGreen();
    }

    Pose2D otosPose = myOtos.getPosition();


    // LOG_DEBUG_THROTTLE("otos", 100, "reading: \t%f, \t%f, \t%f", otosPose.x,
    //                    otosPose.y, otosPose.h);

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
