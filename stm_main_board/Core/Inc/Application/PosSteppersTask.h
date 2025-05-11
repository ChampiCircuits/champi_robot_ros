#ifndef POSSTEPPERS_H
#define POSSTEPPERS_H

#include "Devices/PosStepper.h"

namespace devices
{
    extern PosStepper stepper_opt0;
    // extern PosStepper stepper_opt1;
}


void PosSteppersTaskStart();

#endif //POSSTEPPERS_H
