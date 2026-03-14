#include "Application/Actuators/ActuatorsTask.h"

#include "usart.h"

#include "Application/Modbus/DataStructures.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/Actuators/LiftAndClamp.h"
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
LiftAndClamp liftAndClamp;

void lowerThermometerServo()
{
    devices::scs_servos::set_angle(THERMO_SERVO_ID, THERMO_SERVO_CLOSED, 300);
}

void raiseThermometerServo()
{
    devices::scs_servos::set_angle(THERMO_SERVO_ID, THERMO_SERVO_OPEN, 300);
}


void initEveryThing()
{
    LOG_INFO("act", "Beginning actuators initializing...");
    osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found
    osDelay(1000);

    liftAndClamp.initialization();
    // TODO others

    LOG_INFO("act", "Actuators have been initialized !");
}

void HandleRequest(const ActuatorCommand cmd)
{
    switch (cmd)
    {

    case ActuatorCommand::RESET_ACTUATORS:
        initEveryThing();
        break;
    case ActuatorCommand::STOP_ALL_MOTORS:
        stop_all_actuators_requested = true;
        break;
    case ActuatorCommand::ENABLE_ALL_MOTORS:
        stop_all_actuators_requested = false;
        break;

    case ActuatorCommand::GET_READY:
        break; // TODO needed ?

    case ActuatorCommand::THERMOMETER_LOWER_SERVO:
        lowerThermometerServo();
        break;
    case ActuatorCommand::THERMOMETER_RAISE_SERVO:
        raiseThermometerServo();
        break;
    case ActuatorCommand::TAKE_2_BOXES:
        liftAndClamp.take2Boxes();
        break;
    case ActuatorCommand::BRING_2_BOXES_ON_TOP:
        liftAndClamp.bring2BoxesToTop();
        break;
    case ActuatorCommand::PUT_2_LAST_BOXES_ON_THE_GROUND:
        liftAndClamp.put2LastBoxesOnTheGround();
        break;

    default:
        LOG_ERROR("act", "Unknown Actuator command %d in HandleRequest()", cmd);
        break;
    }
}

void handleManualRequests()
{
    for (int i=0; i < static_cast<size_t>(ActuatorCommand::ACTUATORS_COUNT); i++)
    {
        xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
        ActuatorState actuator_request = static_cast<ActuatorState>(mod_reg::actuators->requests[i]);
        xSemaphoreGive(ModbusH.ModBusSphrHandle);

        if (actuator_request == ActuatorState::REQUESTED)
        {
            ActuatorCommand actuator = static_cast<ActuatorCommand>(i);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_string(actuator).c_str(), to_string(actuator_request).c_str());
            HandleRequest(actuator);
            xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
            mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::DONE);
            xSemaphoreGive(ModbusH.ModBusSphrHandle);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_string(actuator).c_str(), to_string(static_cast<ActuatorState>(mod_reg::actuators->requests[i])).c_str());
        }
    }
}

void ActuatorsTask(void *argument)
{
    initEveryThing();

    LOG_INFO("act", "Starting loop.");
    while (true)
    {
        handleManualRequests();
        osDelay(100);
    }
}

void ActuatorsTaskStart()
{
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
