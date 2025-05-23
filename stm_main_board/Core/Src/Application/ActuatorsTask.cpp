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

bool stop_all_actuators_requested = false;

#define MAX_SERVO_END_ANGLE 155
// max is 270°
#define SERVO_END_LEFT_OPEN_POSITION 0 // TODO regler
#define SERVO_END_LEFT_CLOSE_POSITION MAX_SERVO_END_ANGLE+8
#define SERVO_END_RIGHT_OPEN_POSITION 270
#define SERVO_END_RIGHT_CLOSE_POSITION 270-MAX_SERVO_END_ANGLE

#define SERVO_ARM_UP_POSITION 195
#define SERVO_ARM_DOWN_POSITION 90

#define STEPPER_LOWER_POSITION 0.1

#define SERVO_Y_RIGHT_OUT_POS 270
#define SERVO_Y_RIGHT_TAKE_POS 150
#define SERVO_Y_RIGHT_IN_POS 140
#define SERVO_Y_LEFT_OUT_POS 255
#define SERVO_Y_LEFT_TAKE_POS 140
#define SERVO_Y_LEFT_IN_POS 130

struct ServoY
{
    uint8_t id;
    float angle_out;
    float angle_in;
    float angle_take;
};

ServoY SERVO_Y_LEFT = {ID_SERVO_Y_LEFT, SERVO_Y_LEFT_OUT_POS, SERVO_Y_LEFT_IN_POS, SERVO_Y_LEFT_TAKE_POS};
ServoY SERVO_Y_RIGHT = {ID_SERVO_Y_RIGHT, SERVO_Y_RIGHT_OUT_POS, SERVO_Y_RIGHT_IN_POS, SERVO_Y_RIGHT_TAKE_POS};

#define LOWER_PLANK 0
#define UPPER_PLANK 1


void InitYServos()
{
    devices::scs_servos::set_angle_async(SERVO_Y_LEFT.id, SERVO_Y_LEFT.angle_out, 300);
    devices::scs_servos::set_angle(SERVO_Y_RIGHT.id, SERVO_Y_RIGHT.angle_out, 300);
    devices::scs_servos::set_angle_async(SERVO_Y_LEFT.id, SERVO_Y_LEFT.angle_in, 300);
    devices::scs_servos::set_angle(SERVO_Y_RIGHT.id, SERVO_Y_RIGHT.angle_in, 300);
}
void InitLift()
{
    devices::stepper_opt0.set_zero();
    devices::stepper_opt0.set_goal_sync(0.3);
    bool lift_end_switch_released = HAL_GPIO_ReadPin(D6_GPIO_Port, D6_Pin);

    if (lift_end_switch_released)
    {
        devices::stepper_opt0.set_goal_async(-5.0);
        while (lift_end_switch_released)
        {
            osDelay(10);
            lift_end_switch_released = HAL_GPIO_ReadPin(D6_GPIO_Port, D6_Pin);
        }
        devices::stepper_opt0.set_zero();
        devices::stepper_opt0.set_goal_async(0.0);
    }
}

void InitEverything()
{
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_LEFT, SERVO_END_LEFT_OPEN_POSITION, 300);
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_RIGHT, SERVO_END_RIGHT_OPEN_POSITION, 300);

    devices::scs_servos::set_angle(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 300);

    InitLift();

    osDelay(10);
    devices::scs_servos::set_angle_async(ID_SERVO_BANNER, 90, 300);
    devices::scs_servos::set_angle_async(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 300);
    InitYServos();
}

void TakePlank(int plank) { // !!! lift should be down
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_RIGHT, SERVO_END_RIGHT_OPEN_POSITION, 300); // init au cas ou
    devices::scs_servos::set_angle(ID_SERVO_ARM_END_LEFT, SERVO_END_LEFT_OPEN_POSITION, 300); // init au cas ou

    float plank_height;
    if (plank == LOWER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION;
    }
    else if (plank == UPPER_PLANK) {
        plank_height = STEPPER_LOWER_POSITION + 0.4; // TODO 0.48 avant
    }

    // HALF CLOSED PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300); // avant y'avait un -10.0

    devices::stepper_opt0.set_goal_sync(plank_height);
    // CLOSED TO TAKE PLANK
    //devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);

    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_RIGHT, SERVO_END_RIGHT_CLOSE_POSITION, 300);
    devices::scs_servos::set_angle(ID_SERVO_ARM_END_LEFT, SERVO_END_LEFT_CLOSE_POSITION, 300);

    if (plank == LOWER_PLANK) { // dans ce cas, pas besoin de remonter haut vu qu'on va prendre les conserves
        devices::stepper_opt0.set_goal_async(1.2); // async pour passer dans l'état move plus tôt
    }
    else {
        devices::stepper_opt0.set_goal_async(3.6);
    }
}

void PutPlanks(int layer)
{
    float layer_height;
    if (layer == 1)
        layer_height = STEPPER_LOWER_POSITION;
    else if (layer == 2)
        layer_height = STEPPER_LOWER_POSITION + 3.4;

    // CLOSED TO PUT PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);
    devices::stepper_opt0.set_goal_sync(layer_height);

    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_RIGHT, SERVO_END_RIGHT_OPEN_POSITION, 300);
    devices::scs_servos::set_angle(ID_SERVO_ARM_END_LEFT, SERVO_END_LEFT_OPEN_POSITION, 300);

    //devices::scs_servos::set_angle_async(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION+10.0, 300);

    if (layer == 2) { // in this case we open a bit the arm to not destroy the tower
        devices::scs_servos::set_angle_async(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 300);
    }
}

void TakeCan(struct ServoY servo)
{
    devices::scs_servos::set_angle(servo.id, servo.angle_out, 300);
    devices::stepper_opt0.set_goal_sync(0.37);
    devices::scs_servos::set_angle(servo.id, servo.angle_take, 300);
    devices::stepper_opt0.set_goal_async(0.37+0.6); // async pour partir en move plus tôt
}

void PutCan(int layer, struct ServoY servo)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.4;
    else if (layer==2)
        layer_base_height = 3.6;

    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(servo.id, servo.angle_out, 300);
    // little movement to get the Y inside
    devices::stepper_opt0.set_goal_sync(layer_base_height + 0.55);
    devices::scs_servos::set_angle(servo.id, servo.angle_in, 500);

    devices::stepper_opt0.set_goal_sync(layer_base_height);
    devices::scs_servos::set_angle(servo.id, servo.angle_take+50., 300); // PUSH

    // little movement to get the Y inside
    // les deux async pour partir à l'état suivant plus tôt
    if (layer==2) {
        // devices::stepper_opt0.set_goal_async(layer_base_height + 0.6);
        devices::scs_servos::set_angle_async(servo.id, servo.angle_in, 300);
    }
    else {
        devices::scs_servos::set_angle(servo.id, servo.angle_in, 300);
        devices::stepper_opt0.set_goal_async(STEPPER_LOWER_POSITION); // pour préparer le put lower plank
    }
}

void PutBanner()
{
    devices::scs_servos::set_angle(ID_SERVO_BANNER, 250, 1000);
}

void getReady()
{
    devices::scs_servos::set_angle_async(ID_SERVO_ARM, (SERVO_ARM_UP_POSITION+SERVO_ARM_DOWN_POSITION)/2.0, 300);
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_RIGHT, SERVO_END_RIGHT_OPEN_POSITION, 300); // init au cas ou
    devices::scs_servos::set_angle_async(ID_SERVO_ARM_END_LEFT, SERVO_END_LEFT_OPEN_POSITION, 300); // init au cas ou
    InitLift();
    devices::stepper_opt0.set_goal_async(STEPPER_LOWER_POSITION);
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
        TakeCan(SERVO_Y_LEFT);
        break;
    case ActuatorCommand::TAKE_CANS_RIGHT:
        TakeCan(SERVO_Y_RIGHT);
        break;
    case ActuatorCommand::PUT_CANS_LEFT_LAYER_1:
        PutCan(1, SERVO_Y_LEFT);
        break;
    case ActuatorCommand::PUT_CANS_RIGHT_LAYER_2:
        PutCan(2, SERVO_Y_RIGHT);
        break;
    case ActuatorCommand::RESET_ACTUATORS:
        InitEverything();
        break;
    case ActuatorCommand::STOP_ALL_MOTORS:
        stop_all_actuators_requested = true;
        break;
    case ActuatorCommand::ENABLE_ALL_MOTORS:
        stop_all_actuators_requested = false;
        break;
    case ActuatorCommand::GET_READY:
        getReady();
        break;
    }
}

void ActuatorsTask(void *argument) {

    osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found
    osDelay(1000);
    InitEverything();

    LOG_INFO("act", "Starting loop.");

    while (true)
    {
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
