#include "Application/Actuators/FourSuctionCup.h"

#include "logging.h"
#include "SCServosApp.h"


void FourSuctionCup::setMontagePosition()
{
    // set all in lower cups position
    setCupPosition(0, CUP_SERVO_LOW);
    setCupPosition(1, CUP_SERVO_LOW);
    setCupPosition(2, CUP_SERVO_LOW);
    setCupPosition(3, CUP_SERVO_LOW);
}

void FourSuctionCup::initAllServos()
{
    // set all in high cups position
    setCupPosition(0, CUP_SERVO_RETURN);
    setCupPosition(1, CUP_SERVO_RETURN);
    setCupPosition(2, CUP_SERVO_RETURN);
    setCupPosition(3, CUP_SERVO_RETURN);
}

void FourSuctionCup::lowerCups()
{
    setPumpState(true);
    setCupsPosition(0x0F, CUP_SERVO_LOW);
    osDelay(2000);
    setCupsPosition(0x0F, CUP_SERVO_RETURN);
    osDelay(2000);
}

void FourSuctionCup::raiseCups()
{
    setCupsPosition(0x0F, CUP_SERVO_HIGH);
    osDelay(3000);
}

void FourSuctionCup::getReadyCups()
{
    setCupsPosition(0x0F, CUP_SERVO_LOW_GET_READY);
    osDelay(3000);
}

void FourSuctionCup::letGoCups(uint8_t suction_cups_activation_for_request)
{

    LOG_INFO("4cup", "letGoCups (right to left) mask=0x%02X: cup0=%s cup1=%s cup2=%s cup3=%s",
        suction_cups_activation_for_request,
        (suction_cups_activation_for_request & 0b0001) ? "RETURN" : "LOW",
        (suction_cups_activation_for_request & 0b0010) ? "RETURN" : "LOW",
        (suction_cups_activation_for_request & 0b0100) ? "RETURN" : "LOW",
        (suction_cups_activation_for_request & 0b1000) ? "RETURN" : "LOW");
    // si c'est à true, alors on le retourne, dans ce cas là on envoie en position RETOURNEE, sinon en position LOW
    setCupPosition(0, (suction_cups_activation_for_request & 0b0001) ? CUP_SERVO_RETURN : CUP_SERVO_LOW);
    setCupPosition(1, (suction_cups_activation_for_request & 0b0010) ? CUP_SERVO_RETURN : CUP_SERVO_LOW);
    setCupPosition(2, (suction_cups_activation_for_request & 0b0100) ? CUP_SERVO_RETURN : CUP_SERVO_LOW);
    setCupPosition(3, (suction_cups_activation_for_request & 0b1000) ? CUP_SERVO_RETURN : CUP_SERVO_LOW);
    osDelay(5000);
    setPumpState(false);
}

void FourSuctionCup::setPumpState(bool enable)
{
    // write the GPIO to control the pump relay
    HAL_GPIO_WritePin(PUMP_0_GPIO_Port, PUMP_0_GPIO_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void FourSuctionCup::setCupsPosition(uint8_t mask, float position)
{
    // mask is a bitmask where each bit represents whether the corresponding cup should be activated (1) or not (0).
    for (int i = 0; i < 4; i++)
        if (mask & (1 << i)) setCupPosition(i, position);
}

void FourSuctionCup::setCupPosition(int cup, float position)
{
    int servoID;
    switch (cup)
    {
        case 0: servoID = CUP_0_SERVO_ID; break;
        case 1: servoID = CUP_1_SERVO_ID; break;
        case 2: servoID = CUP_2_SERVO_ID; break;
        case 3: servoID = CUP_3_SERVO_ID; break;
        default:
            {
                LOG_INFO("4cup", "Invalid cup number: %d", cup);
                return; // Invalid cup number
            }
    }
    auto positionStr = (position == CUP_SERVO_LOW) ? "LOW" : (position == CUP_SERVO_HIGH) ? "HIGH" : (position == CUP_SERVO_RETURN) ? "RETURN" : (position == CUP_SERVO_LOW_GET_READY) ? "LOW_GET_READY" : "UNKNOWN";
    if (CUP_INVERTED[cup]) position = CUP_SERVO_MAX_ANGLE - position;
    devices::scs_servos::set_angle_async(servoID, position, 2000);
    osDelay(100);

    LOG_INFO("4cup", "Setting cup %d to position %s (servo angle %.0f, servo ID %d)", cup, positionStr, position, servoID);
}