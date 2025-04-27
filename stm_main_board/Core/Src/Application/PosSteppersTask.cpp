#include "Application/PosSteppersTask.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/Modbus/ModbusTask.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"
#include "tim.h"

namespace devices
{
    PosStepper stepper_opt0;
    // PosStepper stepper_opt1;
}


using namespace devices;

osThreadId_t PosSteppersTaskHandle;
const osThreadAttr_t posSteppersTask_attributes = {
    .name = "pos_steppers_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};


void PosSteppersTask(void *argument) {

    uint32_t start = osKernelGetTickCount();

    while (true)
    {
        stepper_opt0.spin_once(static_cast<double>(POSSTEPPERS_LOOP_PERIOD_MS) / 1000.);

        int time_to_wait = POSSTEPPERS_LOOP_PERIOD_MS - (int)osKernelGetTickCount() + (int)start;
        if (time_to_wait > 0) {
            // avoid negative delay
            osDelay((uint32_t)time_to_wait);
        }
        start = osKernelGetTickCount();
    }
}

void PosSteppersTaskStart() {
    stepper_opt0 = PosStepper(htim17, TIM_CHANNEL_1, DIR_OPT0_GPIO_Port, DIR_OPT0_Pin);
    // stepper_opt1 = PosStepper(htim16, TIM_CHANNEL_1, DIR_OPT1_GPIO_Port, DIR_OPT1_Pin);
    PosSteppersTaskHandle = osThreadNew(PosSteppersTask, NULL, &posSteppersTask_attributes);
}
