//
// Created by etienne on 4/27/25.
//

#ifndef SCSERVOSAPP_H
#define SCSERVOSAPP_H

#include <vector>

#include "Devices/SCServos.h"
#include "stm32h7xx_hal.h"

#define ID_SERVO_ARM_END 7
#define ID_SERVO_ARM 10 // TODO wrong
#define ID_SERVO_Y_FRONT 13
#define ID_SERVO_Y_SIDE 15

namespace devices
{
    namespace scs_servos {

        extern std::vector<uint8_t> ids_servos;
        extern SCServos servos;
        extern bool init_successful;

        int test();
        void set_enable(bool enable);
        void set_angle(uint8_t id, float angle, int ms);
        void set_angle_async(uint8_t id, float angle, int ms);
    }
}

int SCServosApp_Init();

#endif //SCSERVOSAPP_H
