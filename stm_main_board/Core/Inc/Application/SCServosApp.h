//
// Created by etienne on 4/27/25.
//

#ifndef SCSERVOSAPP_H
#define SCSERVOSAPP_H

#define N_SERVOS 5

#define ID_SERVO_ARM_END 7
#define ID_SERVO_ARM 15
#define ID_SERVO_Y_FRONT 13
#define ID_SERVO_Y_SIDE 8
#define ID_SERVO_BANNER 14

#include "cmsis_os2.h"

#include "Devices/SCServos.h"

namespace devices
{
    namespace scs_servos {

        extern uint8_t ids_servos[N_SERVOS];
        extern SCServos servos;
        extern bool init_successful;

        int test();
        void set_enable(bool enable);
        float read_angle(uint8_t id);
        void set_angle(uint8_t id, float angle, int ms);
        void set_angle_async(uint8_t id, float angle, int ms);
        void test_angle(uint8_t id, float angle); // Tests a given angle, then reverts it to original angle.
    }
}

int SCServosApp_Init();

#endif //SCSERVOSAPP_H
