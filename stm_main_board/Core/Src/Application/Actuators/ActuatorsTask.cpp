#include "Application/Actuators/ActuatorsTask.h"

#include "usart.h"

#include "Application/Modbus/DataStructures.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/Actuators/LiftAndClamp.h"
#include "Application/Actuators/BoxesSorter.h"
#include "Application/SCServosApp.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Modbus/hw_actuators.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"

bool stop_all_actuators_requested = false;
LiftAndClamp liftAndClamp;
BoxesSorter boxesSorter;

osThreadId_t ActuatorsTaskHandle;
const osThreadAttr_t actuatorsTask_attributes = {
    .name = "actuators_task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void lowerThermometerServo()
{
    devices::scs_servos::set_angle(THERMO_SERVO_ID, THERMO_SERVO_CLOSED, 300);
}

void raiseThermometerServo()
{
    devices::scs_servos::set_angle_async(THERMO_SERVO_ID, THERMO_SERVO_OPEN, 300);
}

void initEveryThing()
{
    LOG_INFO("act", "Beginning actuators initializing...");
    osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found
    osDelay(1000);

    raiseThermometerServo();
    liftAndClamp.initialize();
    boxesSorter.initialize();

    LOG_INFO("act", "Actuators have been initialized !");
}

void HandleRequest(const ActuatorCommand cmd)
{
    switch (cmd)
    {
    case ActuatorCommand::RESET_ACTUATORS:                  initEveryThing(); break;
    case ActuatorCommand::STOP_ALL_MOTORS:                  stop_all_actuators_requested = true; break;
    case ActuatorCommand::ENABLE_ALL_MOTORS:                stop_all_actuators_requested = false; break;
    case ActuatorCommand::THERMOMETER_LOWER_SERVO:          lowerThermometerServo(); break;
    case ActuatorCommand::THERMOMETER_RAISE_SERVO:          raiseThermometerServo(); break;
    case ActuatorCommand::TAKE_2_BOXES:                     liftAndClamp.take2Boxes(); break;
    case ActuatorCommand::BRING_2_BOXES_ON_TOP:
        if (!boxesSorter.isPusherReady()) boxesSorter.prepareTopPusher(); // defensive: pusher must be retracted first
        liftAndClamp.bring2BoxesToTop();
        break;
    case ActuatorCommand::PUT_2_LAST_BOXES_ON_THE_GROUND:   liftAndClamp.put2LastBoxesOnTheGround(); break;
    case ActuatorCommand::PREPARE_TOP_PUSHER:               boxesSorter.prepareTopPusher(); break;
    case ActuatorCommand::GRAB_AND_SORT_2_BOXES_FROM_LIFT:
        boxesSorter.grabAndSort2BoxesFromLift();
        liftAndClamp.markBoxesGrabbed();
        break;
    case ActuatorCommand::PUSH_2_BOXES_OUT:                 boxesSorter.push2BoxesOut(); break;
    case ActuatorCommand::OPEN_EXIT_RAMP:                   boxesSorter.openExitRamp(); break;

    default:
        LOG_ERROR("act", "Unknown Actuator command %s in HandleRequest()", to_c_str(cmd));
        break;
    }
}

/**
 * Returns true if any actuator command is currently in REQUESTED state (i.e. a manual ROS request is pending).
 * Auto-pipeline must not run while manual requests are waiting.
 */
bool hasManualRequestPending()
{
    for (size_t i = 0; i < static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT); i++)
    {
        xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
        const bool isPending = static_cast<ActuatorState>(mod_reg::actuators->requests[i]) == ActuatorState::REQUESTED;
        xSemaphoreGive(ModbusH.ModBusSphrHandle);
        if (isPending) return true;
    }
    return false;
}

void handleManualRequests(){
    for (size_t i=0; i < static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT); i++)
    {
        xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
        ActuatorState actuator_request = static_cast<ActuatorState>(mod_reg::actuators->requests[i]);
        xSemaphoreGive(ModbusH.ModBusSphrHandle);

        if (actuator_request == ActuatorState::REQUESTED)
        {
            ActuatorCommand actuator = static_cast<ActuatorCommand>(i);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_c_str(actuator), to_c_str(actuator_request));
            HandleRequest(actuator);
            xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
            mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::DONE);
            xSemaphoreGive(ModbusH.ModBusSphrHandle);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_c_str(actuator), to_c_str(static_cast<ActuatorState>(mod_reg::actuators->requests[i])));
        }
    }
}

/**
 * Advances the box pipeline one step forward, starting from the last stage.
 * Pipeline order: prepareTopPusher → bring2BoxesToTop → grabAndSort2BoxesFromLift
 * (push2BoxesOut is always triggered manually by ROS)
 *
 * Preconditions:
 *  - At least 4 boxes must be in the lift (2 are always kept clamped at the bottom).
 *  - prepareTopPusher() must be done before bring2BoxesToTop() (enforced here).
 *  - This function does nothing if a manual ROS request is pending.
 */
void update_elements_pipeline()
{
    if (hasManualRequestPending()) return;

    if (liftAndClamp.hasBoxesReadyAtTop())
    {
        // Stage 3: 2 boxes are at the top of the lift, ready to be grabbed and sorted.
        boxesSorter.grabAndSort2BoxesFromLift();
        liftAndClamp.markBoxesGrabbed();
        // grabAndSort2BoxesFromLift() returns the pusher to READY position,
        // so isPusherReady() == true on the next cycle → stage 2 will fire directly.
    }
    else if (boxesSorter.isPusherReady() && liftAndClamp.boxesInLiftCount > 2)
    {
        // Stage 2: pusher is retracted and there are boxes to move up.
        liftAndClamp.bring2BoxesToTop();
    }
    else if (!boxesSorter.isPusherReady() && liftAndClamp.boxesInLiftCount > 2)
    {
        // Stage 1: pusher is not retracted yet — retract it before the lift can rise.
        boxesSorter.prepareTopPusher();
    }
}

void ActuatorsTask(void *argument)
{
    // initEveryThing();

    LOG_INFO("act", "Starting loop.");
    while (true)
    {
        // handleManualRequests();
        //
        // if (mod_reg::requests->team_color != boxesSorter.getTeamColor())
        //     boxesSorter.setTeamColor(mod_reg::requests->team_color);
        //
        // update_elements_pipeline();

        osDelay(100);
    }
}

void ActuatorsTaskStart()
{
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
