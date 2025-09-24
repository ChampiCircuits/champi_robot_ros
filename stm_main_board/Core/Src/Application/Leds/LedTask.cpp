#include "Application/Leds/LedTask.h"
#include "Application/Leds/animations.h"
#include "Application/Leds/Leds.h"

#include "usart.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Leds/Leds.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "semphr.h"
#include "cmsis_os2.h"



osThreadId_t LedTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "leds_task",
  .stack_size = 1024 * 2,
  .priority = (osPriority_t)osPriorityLow,
};


void LedTask(void *argument) {
  LOG_INFO("led", "Starting loop.");
  clear_Ring();
  osDelay(100);

  while (true) {
    applyLedState({led_holo::color, led_holo::brightness}, LED_HOLO);
    applyLedState({led_otos::color, led_otos::brightness}, LED_OTOS);

     xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
     bool e_stop_pressed = mod_reg::state->e_stop_pressed;
     xSemaphoreGive(ModbusH.ModBusSphrHandle);

     if (e_stop_pressed) {
       BAU_pushed_animation();
     }
     else {
        turning_rainbow_animation();
     }

      //horizontal_lines_animation();
    osDelay(50);
  }
}

void LedTaskStart() {
  LedTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);
}
