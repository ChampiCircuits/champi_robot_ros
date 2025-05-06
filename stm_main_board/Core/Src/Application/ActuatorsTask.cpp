#include "Application/ActuatorsTask.h"

#include "usart.h"

#include "Application/Modbus/ModbusRegister.h"
#include "Application/PosSteppersTask.h"
#include "Application/SCServosApp.h"
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

#define SERVO_END_OPEN_POSITION 270
#define SERVO_END_CLOSE_POSITION 0

#define SERVO_ARM_UP_POSITION 90
#define SERVO_ARM_DOWN_POSITION 220

#define STEPPER_UPPER_POSITION 3.3
#define STEPPER_LOWER_POSITION 0. //0.3

#define SERVO_Y_OUT_POS 225
#define SERVO_Y_IN_POS 145

#define LOWER_PLANK 0
#define UPPER_PLANK 1

void InitArm()
{
    // OPEN THE ARM and END_ARM
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 300);
    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 100);
}

void InitBanner()
{
    devices::scs_servos::set_angle_async(ID_SERVO_BANNER, 90, 1000);
}
void InitYServos()
{
    devices::scs_servos::set_angle_async(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS, 1000);
    devices::scs_servos::set_angle_async(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS, 1000);
}
void InitEverything()
{
    InitBanner();
    devices::stepper_opt0.set_goal(0.0);
    InitArm();
    InitYServos();
}
void RetractArm()
{
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_UP_POSITION, 300);
}
void TakePlank(int plank) { // !!! lift should be down
    float plank_height;
    if (plank == LOWER_PLANK)
        plank_height = STEPPER_LOWER_POSITION;
    else if (plank == UPPER_PLANK)
        plank_height = STEPPER_LOWER_POSITION + 0.2;

    // CLOSED TO TAKE PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);

    devices::stepper_opt0.set_goal(plank_height);
    osDelay(1000);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_CLOSE_POSITION, 1000);
    osDelay(1000);
}

void PutPlanks(int layer)
{
    float layer_height;
    if (layer == 1)
        layer_height = STEPPER_LOWER_POSITION;
    else if (layer == 2)
        layer_height = STEPPER_LOWER_POSITION + 3.0; // TODO + ONE_LAYER_HEIGHT

    // CLOSED TO PUT PLANK
    devices::scs_servos::set_angle(ID_SERVO_ARM, SERVO_ARM_DOWN_POSITION, 300);

    devices::stepper_opt0.set_goal(layer_height);
    osDelay(3000);

    devices::scs_servos::set_angle(ID_SERVO_ARM_END, SERVO_END_OPEN_POSITION, 100);
    osDelay(1000);
}

void TakeCanFront()
{
    devices::stepper_opt0.set_goal(1.0);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_OUT_POS,500);
    devices::stepper_opt0.set_goal(0.);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS,500);
}

void PutCanFront(int layer)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.;
    else if (layer==2)
        layer_base_height = 3.0;

    devices::stepper_opt0.set_goal(layer_base_height);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_OUT_POS,500);

    // little movement to get the Y inside
    devices::stepper_opt0.set_goal(layer_base_height + 0.3);
    osDelay(1000);
    devices::scs_servos::set_angle(ID_SERVO_Y_FRONT, SERVO_Y_IN_POS,500);
    devices::stepper_opt0.set_goal(layer_base_height);
}
void TakeCanSide()
{
    devices::stepper_opt0.set_goal(1.0);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_OUT_POS,500);
    devices::stepper_opt0.set_goal(0.);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS,500);
}
void PutCanSide(int layer)
{
    float layer_base_height;
    if (layer==1)
        layer_base_height = 0.;
    else if (layer==2)
        layer_base_height = 3.0;
    devices::stepper_opt0.set_goal(layer_base_height);
    osDelay(2000);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_OUT_POS,500);

    // little movement to get the Y inside
    devices::stepper_opt0.set_goal(layer_base_height + 0.3);
    osDelay(1000);
    devices::scs_servos::set_angle(ID_SERVO_Y_SIDE, SERVO_Y_IN_POS,500);
    devices::stepper_opt0.set_goal(layer_base_height);
}
void PutBanner()
{
    devices::scs_servos::set_angle(ID_SERVO_BANNER, 250, 1000);
}

void ActuatorsTask(void *argument) {

    SCServosApp_Init(); // Reminder: blocking until the servos are found

    InitEverything();
    osDelay(5000);

    LOG_INFO("act", "Starting loop.");

    {
        //move in front
        TakePlank(LOWER_PLANK);
        osDelay(4000);
        //move to the right two cans
        TakeCanFront();
        osDelay(4000);
        //move and rotate to the left two cans
        TakeCanSide();
        osDelay(4000);
    }

    //move somewhere
    {
        PutCanFront(1);
        osDelay(4000);
        PutPlanks(1);
        TakePlank(UPPER_PLANK);
        osDelay(4000);
        //rotate
        PutCanSide(2);
        PutPlanks(2);
    }

    devices::stepper_opt0.set_goal(3.0);

    while (true) {
        // devices::scs_servos::set_enable(false);
        // auto angle = devices::scs_servos::read_angle(ID_SERVO_Y_FRONT);
        // LOG_INFO("act", "Servo Y front angle: %f", angle);
        osDelay(1000);
    }
}

void ActuatorsTaskStart() {
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
