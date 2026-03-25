#ifndef LED_STATUS_H
#define LED_STATUS_H

#include "motion.h"

void ledStatusInit();
void ledStatusApply(MotionState state, Team latched_team);

#endif // LED_STATUS_H

