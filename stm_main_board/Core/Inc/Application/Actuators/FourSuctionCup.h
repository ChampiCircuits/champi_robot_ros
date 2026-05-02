#ifndef STM_MAIN_BOARD_4SUCTIONCUP_H
#define STM_MAIN_BOARD_4SUCTIONCUP_H

#include <cstdint>
#include "main.h"

class FourSuctionCup
{
public:
    FourSuctionCup(int servo_0_ID, int servo_1_ID, int servo_2_ID, int servo_3_ID,
                   GPIO_TypeDef* end_switch_gpio_port, uint16_t end_switch_gpio_pin)
    {
        CUP_0_SERVO_ID = servo_0_ID;
        CUP_1_SERVO_ID = servo_1_ID;
        CUP_2_SERVO_ID = servo_2_ID;
        CUP_3_SERVO_ID = servo_3_ID;
        PUMP_0_GPIO_Port = end_switch_gpio_port;
        PUMP_0_GPIO_Pin  = end_switch_gpio_pin;
    }

    void setMontagePosition();
    void initAllServos();

    void lowerCups(uint8_t suction_cups_activation_for_request);
    void raiseCups(uint8_t suction_cups_activation_for_request);
    void letGoCups(uint8_t suction_cups_activation_for_request);
    void setPumpState(bool enable);

    uint8_t CUP_0_SERVO_ID = 0;
    uint8_t CUP_1_SERVO_ID = 0;
    uint8_t CUP_2_SERVO_ID = 0;
    uint8_t CUP_3_SERVO_ID = 0;

    GPIO_TypeDef* PUMP_0_GPIO_Port = nullptr;
    uint16_t      PUMP_0_GPIO_Pin  = 0;

private:
    void setCupsPosition(uint8_t mask, float position);
    void setCupPosition(int cup, float position);

    static constexpr int CUP_SERVO_LOW    = 90;   // ° [0,270] // TODO
    static constexpr int CUP_SERVO_HIGH   = 180;  // ° [0,270] // TODO
    static constexpr int CUP_SERVO_RETURN = 250;  // ° [0,270] // TODO
};

#endif //STM_MAIN_BOARD_4SUCTIONCUP_H
