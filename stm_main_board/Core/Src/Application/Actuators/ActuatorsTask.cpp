#include "Application/Actuators/ActuatorsTask.h"

#include "usart.h"

#include "Application/Modbus/DataStructures.h"
#include "Application/Modbus/ModbusRegister.h"
#include "Application/SCServosApp.h"
#include "Application/Modbus/ModbusTask.h"
#include "Application/Modbus/hw_actuators.h"
#include "Config/Config.h"
#include "Util/logging.h"

#include "cmsis_os2.h"
#include "semphr.h"
#include "Actuators/FourSuctionCup.h"

bool stop_all_actuators_requested = false;

// Masks saved at LOWER time, applied at LET_GO time, then reset.
// This ensures the correct color mask is used even if the Modbus register changes between the two commands.
static uint8_t pending_left_mask  = 0;
static uint8_t pending_right_mask = 0;

// CUPS are from right to left, from the point of view of the robot. So cup0 is the rightmost, cup3 the leftmost.
uint8_t LEFT_ARM_0_SERVO_ID = 5;
uint8_t LEFT_ARM_1_SERVO_ID = -1; // TODO REMOVED
uint8_t LEFT_ARM_2_SERVO_ID = 13;
uint8_t LEFT_ARM_3_SERVO_ID = 17;


uint8_t RIGHT_ARM_0_SERVO_ID = 8;
uint8_t RIGHT_ARM_1_SERVO_ID = 18;
uint8_t RIGHT_ARM_2_SERVO_ID = 6;
uint8_t RIGHT_ARM_3_SERVO_ID = 9;

FourSuctionCup left_arm(LEFT_ARM_0_SERVO_ID,LEFT_ARM_1_SERVO_ID,LEFT_ARM_2_SERVO_ID,LEFT_ARM_3_SERVO_ID, D0_GPIO_Port, D0_Pin);
FourSuctionCup right_arm(RIGHT_ARM_0_SERVO_ID,RIGHT_ARM_1_SERVO_ID,RIGHT_ARM_2_SERVO_ID,RIGHT_ARM_3_SERVO_ID, D1_GPIO_Port, D1_Pin);

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

void setServoInContinousRotation(const uint8_t servo_id)
{
    // Set wheel (infinite rotation) mode: both angle limits to 0
    devices::scs_servos::servos.WriteLimitAngle(servo_id, 0, 0);
    osDelay(1000);
}

void initEveryThing()
{
    LOG_INFO("act", "Beginning actuators initializing...");

    // {// TEST PUMPS
    //     // left_arm.setPumpState(true);
    //     right_arm.setPumpState(true);
    //     LOG_INFO("act", "Pumps should be ON for 3 seconds...");
    //     osDelay(3000);
    //     // left_arm.setPumpState(false);
    //     right_arm.setPumpState(false);
    //     LOG_INFO("act", "Pumps should be OFF now.");
    // }
    
    // osDelay(3000);
    SCServosApp_Init(); // Reminder: blocking until the servos are found

    // while(1){}
    
    { // MOUNTING ONLY
        // left_arm.setMontagePosition();
        // right_arm.setMontagePosition();
        // LOG_INFO("act", "Montage position set for both arms. Please mount the arms in the LOW position");
        // while (1) {}
    }

    left_arm.initAllServos();
    right_arm.initAllServos();
    raiseThermometerServo();

    LOG_INFO("act", "Actuators have been initialized !");
}

void HandleRequest(const ActuatorCommand cmd,
    uint8_t left_suction_cups_activation_for_request,
    uint8_t right_suction_cups_activation_for_request)
{
    switch (cmd)
    {
    case ActuatorCommand::RESET_ACTUATORS:                  initEveryThing(); break;
    case ActuatorCommand::STOP_ALL_MOTORS:                  stop_all_actuators_requested = true; break;
    case ActuatorCommand::ENABLE_ALL_MOTORS:                stop_all_actuators_requested = false; break;
    case ActuatorCommand::GET_READY: break;
    case ActuatorCommand::THERMOMETER_LOWER_SERVO:          lowerThermometerServo(); break;
    case ActuatorCommand::THERMOMETER_RAISE_SERVO:          raiseThermometerServo(); break;

// TODO faire pareil pour le LEFT

    case ActuatorCommand::STORE_PENDING_MASK:
        pending_right_mask = right_suction_cups_activation_for_request; // save for LET_GO
        LOG_INFO("act", "[RIGHT ARM] STORE: saved pending_right_mask=0x%02X (cups from right to left: %d%d%d%d)",
            pending_right_mask,
            (pending_right_mask >> 0) & 1, (pending_right_mask >> 1) & 1,
            (pending_right_mask >> 2) & 1, (pending_right_mask >> 3) & 1);
        break;
    case ActuatorCommand::LOWER_RIGHT_ARM:
        
        LOG_INFO("act", "[RIGHT ARM] Lowering ALL 4 cups...");
        right_arm.lowerCups();
        // osDelay(3000);
        // LOG_INFO("act", "[RIGHT ARM] Raising ALL 4 cups...");
        // right_arm.raiseCups();
        LOG_INFO("act", "[RIGHT ARM] LOWER done.");
        break;
    case ActuatorCommand::LET_GO_ELEMENTS_RIGHT_ARM:
        LOG_INFO("act", "[RIGHT ARM] LET_GO: applying pending_right_mask=0x%02X (cups from right to left to return: %d%d%d%d)",
            pending_right_mask,
            (pending_right_mask >> 0) & 1, (pending_right_mask >> 1) & 1,
            (pending_right_mask >> 2) & 1, (pending_right_mask >> 3) & 1);
        right_arm.letGoCups(pending_right_mask); // use mask saved at LOWER time
        LOG_INFO("act", "[RIGHT ARM] letGoCups done, resetting pending_right_mask.");
        pending_right_mask = 0;
        LOG_INFO("act", "[RIGHT ARM] LET_GO done. Now putting back all cups to RETURN position...");
        osDelay(1500);
        right_arm.initAllServos();
        break;

    case ActuatorCommand::GET_READY_RIGHT_ARM:
        LOG_INFO("act", "[RIGHT ARM] GET_READY");
        right_arm.getReadyCups();
        LOG_INFO("act", "[RIGHT ARM] getReadyCups done.");
        // right_arm.initAllServos();
        LOG_INFO("act", "[RIGHT ARM] GET_READY done.");
        break;

    case ActuatorCommand::PUMPS_ON:
        LOG_INFO("act", "[PUMPS] Turning ON left and right pumps");
        left_arm.setPumpState(true);
        right_arm.setPumpState(true);
        break;

    case ActuatorCommand::PUMPS_OFF:
        LOG_INFO("act", "[PUMPS] Turning OFF left and right pumps");
        left_arm.setPumpState(false);
        right_arm.setPumpState(false);
        break;

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

        uint8_t left_suction_cups_activation_for_request =
            (mod_reg::actuators->left_suction_cup_0_activation & 1) |
            ((mod_reg::actuators->left_suction_cup_1_activation & 1) << 1) |
            ((mod_reg::actuators->left_suction_cup_2_activation & 1) << 2) |
            ((mod_reg::actuators->left_suction_cup_3_activation & 1) << 3);
        uint8_t right_suction_cups_activation_for_request =
            (mod_reg::actuators->right_suction_cup_0_activation & 1) |
            ((mod_reg::actuators->right_suction_cup_1_activation & 1) << 1) |
            ((mod_reg::actuators->right_suction_cup_2_activation & 1) << 2) |
            ((mod_reg::actuators->right_suction_cup_3_activation & 1) << 3);

        right_suction_cups_activation_for_request = 0; // TODO FOR NOW
        left_suction_cups_activation_for_request = 0;

        xSemaphoreGive(ModbusH.ModBusSphrHandle);

        if (actuator_request == ActuatorState::REQUESTED)
        {
            ActuatorCommand actuator = static_cast<ActuatorCommand>(i);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_c_str(actuator), to_c_str(actuator_request));
            HandleRequest(actuator, left_suction_cups_activation_for_request, right_suction_cups_activation_for_request);
            xSemaphoreTake((QueueHandle_t)ModbusH.ModBusSphrHandle, portMAX_DELAY);
            mod_reg::actuators->requests[i] = static_cast<uint8_t>(ActuatorState::DONE);
            xSemaphoreGive(ModbusH.ModBusSphrHandle);
            LOG_INFO("act", "[MANUAL] Requested actuator %s to state %s", to_c_str(actuator), to_c_str(static_cast<ActuatorState>(mod_reg::actuators->requests[i])));
        }
    }
}

void ActuatorsTask(void *argument)
{
    initEveryThing();

    LOG_INFO("act", "Starting loop.");
    while (true)
    {
        // If the blue USER button (B1, PC13, active LOW) is held at startup → enter montage mode forever
        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
        {
            LOG_INFO("act", "USER button pressed  → entering MONTAGE POSITION mode (reset to exit)");
            right_arm.setMontagePosition();
            left_arm.setMontagePosition();
            LOG_INFO("act", "Montage position set. System halted.");
            while (1) { osDelay(1000); }
        }

        handleManualRequests(); // TODO
        osDelay(100);
    }
}

void ActuatorsTaskStart()
{
    ActuatorsTaskHandle = osThreadNew(ActuatorsTask, NULL, &actuatorsTask_attributes);
}
