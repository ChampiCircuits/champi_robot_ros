#include "Application/ActuatorsTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/PosSteppersTask.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"


osThreadId_t ActuatorsTaskHandle;
const osThreadAttr_t actuatorsTask_attributes = {
    .name = "actuators_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void ActuatorsTask(void *argument) {

    LOG_INFO("act", "Starting loop.");

    while (true) {
        // devices::stepper_opt0.set_goal(4.);
        // osDelay(2000);
        // devices::stepper_opt0.set_goal(0.);
        // osDelay(2000);
    }
}

void ActuatorsTaskStart() {
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
