#include "Application/OtosTask.h"

#include "Config/Config.h"
#include "Devices/QwiicOTOS.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"

#include "i2c.h"

#include "cmsis_os2.h"
#include "semphr.h"



osThreadId_t OtosTaskHandle;
const osThreadAttr_t otosTask_attributes = {
  .name = "holo_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


void OtosTask(void *argument)
{
   QwiicOTOS myOtos(&hi2c4, 0x17);

    while (!myOtos.isConnected()) {
//    	printf("otos not connected\n"); // TODO
        osDelay(1000);
    }

    if (!myOtos.selfTest()) {
    	// ERROR WITH OTOS
        osDelay(1000000000); // TODO better
    }
    
    myOtos.setAngularScalar(OTOS_ANGULAR_SCALAR);
    myOtos.setLinearScalar(OTOS_LINEAR_SCALAR);

    myOtos.setOffset({OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H});

    myOtos.calibrateImu();
		osDelay(10);

    myOtos.resetTracking();
		osDelay(10);
    
    // uint32_t start = osKernelGetTickCount();

    while(true)
    {
      Pose2D otosPose = myOtos.getPosition();
      xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
      *mod_reg::otos_pose = {otosPose.x, otosPose.y, otosPose.h}; // TODO check units
      xSemaphoreGive(ModbusH.ModBusSphrHandle);

      // TODO check freq of otos
      // https://community.st.com/t5/stm32cubeide-mcus/freertos-cmsis-v2-osdelayuntil-go-to-hardfaulhandler-only-with/td-p/263884
      // osDelayUntil(start + CONTROL_LOOP_PERIOD_MS); // TODO vérifier que la freq est OK
      osDelay(10); // TODO substract time taken by the loop
      // start = osKernelGetTickCount();
    }
}

void OtosTaskStart() {
  OtosTaskHandle = osThreadNew(OtosTask, NULL, &otosTask_attributes);
}