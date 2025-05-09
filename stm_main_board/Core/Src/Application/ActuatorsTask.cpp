#include "Application/ActuatorsTask.h"

#include "usart.h"

#include "Application/Modbus/DataStructures.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/PosSteppersTask.h"
#include "Application/SCServosApp.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Modbus/hw_actuators.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"


osThreadId_t ActuatorsTaskHandle;
const osThreadAttr_t actuatorsTask_attributes = {
    .name = "actuators_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

#define SERVO_END_OPEN_POSITION 270
#define SERVO_END_CLOSE_POSITION 0

#define SERVO_ARM_UP_POSITION 90
#define SERVO_ARM_DOWN_POSITION 215

#define STEPPER_UPPER_POSITION 3.3
#define STEPPER_LOWER_POSITION 0. //0.3

#define SERVO_Y_OUT_POS 225
#define SERVO_Y_IN_POS 155

#define LOWER_PLANK 0
#define UPPER_PLANK 1

void InitArm()
{
    // OPEN THE ARM and END_ARM
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 1000);
    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 1000);
}

void InitBanner()
{
    devices::scs_servos::set_angle_async(ID_SERVO_BANNER, 90, 1000);
}
void InitYServos()
{
//    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_OUT_POS, 1000);
//    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_OUT_POS, 1000);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS, 1000);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS, 1000);
}
void InitEverything()
{
    devices::scs_servos::set_angle(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 500);
    devices::stepper_opt0.set_zero();
    devices::stepper_opt0.set_goal_sync(-0.2); // TODO not useful?
    devices::stepper_opt0.set_zero();
    InitBanner();
    InitArm();
    InitYServos();
}
void RetractArm()
{
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 300);
}
void TakePlank(int plank) { // !!! lift should be down
    float plank_height;
    if (plank == LOWER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION;
    }
    else if (plank == UPPER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION + 0.5;
    }

    devices::stepper_opt0.set_goal_sync(plank_height);
    osDelay(1000);

    // CLOSED TO TAKE PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_CLOSE_POSITION, 1000);
    osDelay(1000);

    devices::stepper_opt0.set_goal_sync(plank_height + 1.0);
}

void PutPlanks(int layer)
{
    float layer_height;
    if (layer == 1)
        layer_height = STEPPER_LOWER_POSITION + 0.15;
    else if (layer == 2)
        layer_height = STEPPER_LOWER_POSITION + 3.3; // TODO + ONE_LAYER_HEIGHT

    // CLOSED TO PUT PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);

    devices::stepper_opt0.set_goal_sync(layer_height);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 100);
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION-10.0, 300);

    if (layer == 2) { // in this case we open a bit the arm to not destroy the tower
        devices::scs_servos::set_angle(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 300);
    }
}

void TakeCanFront()
{
    devices::stepper_opt0.set_goal_sync(1.0);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_OUT_POS,500);
    devices::stepper_opt0.set_goal_sync(0.37);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS,500);

    // juste pour lever au-dessus du sol
    devices::stepper_opt0.set_goal_sync(1.0);
}

void PutCanFront(int layer)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.4;
    else if (layer==2)
        layer_base_height = 3.0; // TODO neverused

    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_OUT_POS,500);

    // little movement to get the Y inside
    devices::stepper_opt0.set_goal_sync(layer_base_height + 0.5);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS,500);
    // devices::stepper_opt0.set_goal_sync(layer_base_height);
}
void TakeCanSide()
{
    devices::stepper_opt0.set_goal_sync(1.0);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_OUT_POS,500);
    devices::stepper_opt0.set_goal_sync(0.37);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS,500);

    // juste pour lever au-dessus du sol
    // devices::stepper_opt0.set_goal_sync(1.0); // finalement non, on monte plus haut qu'une planche

    devices::stepper_opt0.set_goal_sync(3.6);
}

void PutCanSide(int layer)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.4;
    else if (layer==2)
        layer_base_height = 3.6;
    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_OUT_POS,500);

    // little movement to get the Y inside
    devices::stepper_opt0.set_goal_sync(layer_base_height + 0.4);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS,500);
    // devices::stepper_opt0.set_goal_sync(layer_base_height);
}
void PutBanner()
{
    devices::scs_servos::set_angle(ID_SERVO_BANNER, 250, 1000);
}

void HandleRequest(ActuatorCommand cmd) {
    switch (cmd) {
    case ActuatorCommand::PUT_BANNER:
        PutBanner();
        break;
    case ActuatorCommand::TAKE_LOWER_PLANK:
        TakePlank(LOWER_PLANK);
        break;
    case ActuatorCommand::TAKE_UPPER_PLANK:
        TakePlank(UPPER_PLANK);
        break;
    case ActuatorCommand::PUT_LOWER_PLANK_LAYER_1:
        PutPlanks(1);
        break;
    case ActuatorCommand::PUT_UPPER_PLANK_LAYER_2:
        PutPlanks(2);
        break;
    case ActuatorCommand::TAKE_CANS_FRONT:
        TakeCanFront();
        break;
    case ActuatorCommand::TAKE_CANS_SIDE:
        TakeCanSide();
        break;
    case ActuatorCommand::PUT_CANS_FRONT_LAYER_1:
        PutCanFront(1);
        break;
    case ActuatorCommand::PUT_CANS_SIDE_LAYER_2:
        PutCanSide(2);
        break;
    case ActuatorCommand::RESET_ACTUATORS:
        InitEverything();
        break;
    case ActuatorCommand::PUT_CANS_SIDE_LAYER_1:
        PutCanSide(1);
        break;
    }
}

void ActuatorsTask(void *argument) {

    osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found
    osDelay(1000);
    InitEverything();
    osDelay(1000);

    LOG_INFO("act", "Starting loop.");

    while (true) {

        for (int i=0; i < ACTUATORS_COUNT; i++)
        {
            xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
            ActuatorState actuator_request = static_cast<ActuatorState>(mod_reg::actuators->requests[i]);
            xSemaphoreGive(ModbusH.ModBusSphrHandle);

            if (actuator_request == ActuatorState::REQUESTED)
            {
                ActuatorCommand actuator = static_cast<ActuatorCommand>(i);
                LOG_INFO("act", "Actuator requested is %s to state %s", to_string(actuator).c_str(), to_string(actuator_request).c_str());
                HandleRequest(actuator);
                xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
                mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::DONE);
                xSemaphoreGive(ModbusH.ModBusSphrHandle);
                LOG_INFO("act", "Actuator requested is %s to state %s", to_string(actuator).c_str(), to_string(static_cast<ActuatorState>(mod_reg::actuators->requests[i])).c_str());
            }
        }
        // LOG_INFO("act", "Actuator requested is %s to state %s", to_string(static_cast<ActuatorCommand>(2)).c_str(), to_string(mod_reg::requests->actuators_state[2]).c_str());

        osDelay(100);
    }

}

void ActuatorsTaskStart() {
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
