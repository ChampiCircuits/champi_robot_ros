#include "Application/HoloDrive/HoloDriveTask.h"

#include "Config/Config.h"
#include "Devices/StepperTimer.h"
#include "Application/HoloDrive/HoloDrive.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"

#include "cmsis_os2.h"
#include "tim.h"
#include "semphr.h"



// Macro to convert value from # define WHEEL_B_TIMER 3 to htim3 (or any value from 1 to 16)
#define CONCAT(a, b) a ## b
#define CONCAT2(a, b) CONCAT(a, b)

#define WHEEL_B_TIMER_HANDLE CONCAT2(htim, WHEEL_B_TIMER_NUMBER)
#define WHEEL_L_TIMER_HANDLE CONCAT2(htim, WHEEL_L_TIMER_NUMBER)
#define WHEEL_R_TIMER_HANDLE CONCAT2(htim, WHEEL_R_TIMER_NUMBER)

osThreadId_t holoDriveTaskHandle;
const osThreadAttr_t holoDriveTask_attributes = {
    .name = "holo_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };


void HoloDriveTask(void *argument)
{
    StepperTimer stepper0(WHEEL_B_TIMER_HANDLE, WHEEL_B_TIMER_CHANNEL, WHEEL_B_DIR_GPIO_PORT, WHEEL_B_DIR_GPIO_PIN);
    StepperTimer stepper1(WHEEL_L_TIMER_HANDLE, WHEEL_L_TIMER_CHANNEL, WHEEL_L_DIR_GPIO_PORT, WHEEL_L_DIR_GPIO_PIN);
    StepperTimer stepper2(WHEEL_R_TIMER_HANDLE, WHEEL_R_TIMER_CHANNEL, WHEEL_R_DIR_GPIO_PORT, WHEEL_R_DIR_GPIO_PIN);

    HoloDrive holoDrive(stepper0, stepper1, stepper2);

    // We wait for the config to be set by the master
    while (!mod_reg::stm_config->is_set) {
        osDelay(100);
    }

    holoDrive.set_config(mod_reg::stm_config->holo_drive_config);

//    uint32_t start = osKernelGetTickCount();

    while(true)
    {
        xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
        Vector3 cmd = *mod_reg::cmd_vel;
         holoDrive.set_cmd_vel(cmd);
         xSemaphoreGive(ModbusH.ModBusSphrHandle);

         holoDrive.spin_once_motors_control();

        xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
         *mod_reg::measured_vel = holoDrive.get_current_vel();
        xSemaphoreGive(ModbusH.ModBusSphrHandle);

//      // https://community.st.com/t5/stm32cubeide-mcus/freertos-cmsis-v2-osdelayuntil-go-to-hardfaulhandler-only-with/td-p/263884
//      osDelayUntil(start + CONTROL_LOOP_PERIOD_MS); // TODO vérifier que la freq est OK
        osDelay(10); // TODO substract time taken by the loop
//      start = osKernelGetTickCount();
    }
}

void HoloDriveTaskStart() {
    holoDriveTaskHandle = osThreadNew(HoloDriveTask, NULL, &holoDriveTask_attributes);
}
