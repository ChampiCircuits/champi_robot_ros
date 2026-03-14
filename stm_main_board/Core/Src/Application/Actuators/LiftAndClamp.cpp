#include "Application/Actuators/LiftAndClamp.h"

#include "Application/SCServosApp.h"
#include "cmsis_os2.h"
#include "Util/logging.h"

/**
 * Lift and CLAMP actuators :
 * - LIFT = 1 stepper + 1 limit switch to init position
 * - CLAMP = 1 servo to clamp the NutBox
 */

void LiftAndClamp::take2Boxes()
{
    LOG_INFO("liftClamp", "Taking 2 boxes...");

    // Go to the top of the boxes we want to take
    _liftGoToPosition(LIFT_BOTTOM_POSITION + BOX_HEIGHT);
    // Release our stack of boxes on top
    _releaseClamp();
    _liftGoToBottom();
    _closeClamp();

    // Always go 1.5 increment higher to avoid dragging boxes on the ground
    // AND avoid hitting other boxes on the floor
    _liftGoToPosition(LIFT_BOTTOM_POSITION + BOX_HEIGHT * 1.5f);

    boxesCount += 2;
    LOG_INFO("liftClamp", "Took 2 boxes !");
}

void LiftAndClamp::bring2BoxesToTop()
{
    _closeClamp();

    // !! Must have already 4 boxes
    // The 2 boxes on top of the stack which are not clamped will then be taken
    // Then we can go just underneath
    const float heightOfStackInLift = BOX_HEIGHT * (boxesCount / 2.0f);
    _liftGoToPosition(LIFT_TOP_POSITION - heightOfStackInLift + BOX_HEIGHT);
}

void LiftAndClamp::put2LastBoxesOnTheGround()
{
    _liftGoToBottom();
    _releaseClamp();
    _liftGoToPosition(LIFT_BOTTOM_POSITION + BOX_HEIGHT * 1.5f);
}

void LiftAndClamp::initialization()
{
    LOG_INFO("liftClamp", "Initializing lift and clamp...");
    _initLift();
    _initClamp();
    LOG_INFO("liftClamp", "Initialized lift and clamp !")
}

void LiftAndClamp::_closeClamp(const bool async)
{
    LOG_INFO("liftClamp", "Closing clamp (%s)...", async ? "async" : "sync");
    if (async)
        devices::scs_servos::set_angle_async(CLAMP_SERVO_ID, CLAMP_SERVO_CLOSED, 300);
    else
        devices::scs_servos::set_angle(CLAMP_SERVO_ID, CLAMP_SERVO_CLOSED, 300);
}

void LiftAndClamp::_releaseClamp(const bool async)
{
    LOG_INFO("liftClamp", "Releasing clamp (%s)...", async ? "async" : "sync");
    if (async)
        devices::scs_servos::set_angle_async(CLAMP_SERVO_ID, CLAMP_SERVO_OPEN, 300);
    else
        devices::scs_servos::set_angle(CLAMP_SERVO_ID, CLAMP_SERVO_OPEN, 300);
}

void LiftAndClamp::_liftGoToPosition(float position, bool async)
{
    if (async)
        devices::stepper_opt0.set_goal_async(position);
    else
        devices::stepper_opt0.set_goal_sync(position);
}

void LiftAndClamp::_initClamp()
{
    LOG_INFO("liftClamp", "Initializing clamp...");
    _releaseClamp(true);
    LOG_INFO("liftClamp", "Initialized clamp !");
}

void LiftAndClamp::_initLift()
{
    LOG_INFO("liftClamp", "Initializing lift...");
    devices::stepper_opt0.set_max_speed(LIFT_INIT_SPEED);
    devices::stepper_opt0.set_max_accel(LIFT_INIT_ACCEL);

    _homingLift();

    devices::stepper_opt0.set_max_speed(LIFT_BASE_SPEED);
    devices::stepper_opt0.set_max_accel(LIFT_BASE_ACCEL);
    LOG_INFO("liftClamp", "Initialized lift !");
}

void LiftAndClamp::_homingLift()
{
    devices::stepper_opt0.set_zero();
    devices::stepper_opt0.set_goal_sync(0.3);

    bool lift_end_switch_released = HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_GPIO_Pin);
    if (lift_end_switch_released)
    {
        devices::stepper_opt0.set_goal_async(-5.0);
        while (lift_end_switch_released)
        {
            osDelay(10);
            lift_end_switch_released = HAL_GPIO_ReadPin(END_SWITCH_GPIO_Port, END_SWITCH_GPIO_Pin);
            LOG_INFO_THROTTLE("liftClamp", 10, "homing lift...");
        }
        devices::stepper_opt0.set_zero();
        devices::stepper_opt0.set_goal_async(0.0);
    }
}
