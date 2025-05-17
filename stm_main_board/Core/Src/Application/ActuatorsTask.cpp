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
#define SERVO_ARM_DOWN_POSITION 205

#define STEPPER_UPPER_POSITION 3.3
#define STEPPER_LOWER_POSITION 0.05

#define SERVO_Y_OUT_POS 225
#define SERVO_Y_TAKE_POS 160
#define SERVO_Y_IN_POS 140

#define LOWER_PLANK 0
#define UPPER_PLANK 1


void InitYServos()
{
    devices::scs_servos::set_angle_async(ID_SERVO_Y_LEFT, SERVO_Y_OUT_POS, 100);
    devices::scs_servos::set_angle(ID_SERVO_Y_RIGHT, SERVO_Y_OUT_POS, 100);
    devices::scs_servos::set_angle_async(ID_SERVO_Y_LEFT, SERVO_Y_IN_POS, 100);
    devices::scs_servos::set_angle(ID_SERVO_Y_RIGHT, SERVO_Y_IN_POS, 100);
}
void InitEverything()
{
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 100);
    devices::scs_servos::set_angle(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 100);
    devices::stepper_opt0.set_goal_sync(0.0);
    devices::stepper_opt0.set_zero();
    osDelay(1);
    devices::scs_servos::set_angle_async(ID_SERVO_BANNER, 90, 100);
    devices::scs_servos::set_angle_async(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 1000);
    InitYServos();
}

void TakePlank(int plank) { // !!! lift should be down
    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 500); // init au cas ou

    float plank_height;
    if (plank == LOWER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION;
    }
    else if (plank == UPPER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION + 0.48;
    }

    // HALF CLOSED PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION-10.0, 100);

    devices::stepper_opt0.set_goal_sync(plank_height);

    // CLOSED TO TAKE PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 100);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_CLOSE_POSITION, 1000);

    if (plank == LOWER_PLANK) { // dans ce cas, pas besoin de remonter haut vu qu'on va prendre les conserves
        devices::stepper_opt0.set_goal_async(1.2); // async pour passer dans l'état move plus tôt
    }
    else {
        devices::stepper_opt0.set_goal_sync(3.6);
    }
}

void PutPlanks(int layer)
{
    float layer_height;
    if (layer == 1)
        layer_height = STEPPER_LOWER_POSITION;
    else if (layer == 2)
        layer_height = STEPPER_LOWER_POSITION + 3.4; // TODO + ONE_LAYER_HEIGHT

    // CLOSED TO PUT PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);
    devices::stepper_opt0.set_goal_sync(layer_height);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 100);
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION-10.0, 300);

    if (layer == 2) { // in this case we open a bit the arm to not destroy the tower
        devices::scs_servos::set_angle(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 300);
    }
}

void TakeCan(int id_servo_which_side)
{
    // devices::stepper_opt0.set_goal_sync(1.2);
    devices::scs_servos::set_angle(id_servo_which_side, SERVO_Y_OUT_POS,200);
    devices::stepper_opt0.set_goal_sync(0.37);
    devices::scs_servos::set_angle(id_servo_which_side, SERVO_Y_TAKE_POS,200);

    devices::stepper_opt0.set_goal_async(0.37+0.6); // async pour partir en move plus tôt
}

void PutCan(int layer, int id_servo_which_side)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.4;
    else if (layer==2)
        layer_base_height = 3.6;

    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(id_servo_which_side, SERVO_Y_OUT_POS,200);

    // little movement to get the Y inside
    devices::stepper_opt0.set_goal_sync(layer_base_height + 0.6);
    devices::scs_servos::set_angle(id_servo_which_side, SERVO_Y_IN_POS,200);

    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(id_servo_which_side, 180,200);

    // little movement to get the Y inside
    // les deux async pour partir à l'état suivant plus tôt
    if (layer==2) {
        // devices::stepper_opt0.set_goal_async(layer_base_height + 0.6);
        devices::scs_servos::set_angle_async(id_servo_which_side, SERVO_Y_IN_POS,200);
    }
    else {
        devices::scs_servos::set_angle(id_servo_which_side, SERVO_Y_IN_POS,200);
        devices::stepper_opt0.set_goal_async(STEPPER_LOWER_POSITION); // pour préparer le put lower plank
    }
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
    case ActuatorCommand::TAKE_CANS_LEFT:
        TakeCan(ID_SERVO_Y_LEFT);
        break;
    case ActuatorCommand::TAKE_CANS_RIGHT:
        TakeCan(ID_SERVO_Y_RIGHT);
        break;
    case ActuatorCommand::PUT_CANS_LEFT_LAYER_1:
        PutCan(1, ID_SERVO_Y_LEFT);
        break;
    case ActuatorCommand::PUT_CANS_RIGHT_LAYER_2:
        PutCan(2, ID_SERVO_Y_RIGHT);
        break;
    case ActuatorCommand::RESET_ACTUATORS:
        InitEverything();
        break;
    }
}

void ActuatorsTask(void *argument) {

    osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found
    osDelay(1000);
    InitEverything();

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
