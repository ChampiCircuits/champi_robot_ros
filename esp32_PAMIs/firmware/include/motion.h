#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

enum class MotionState : uint8_t {
    WAITING_TIRETTE = 0,
    START_DELAY,
    RUNNING,
    PAUSED_OBSTACLE,
    COMPLETED,
    FAULT,
};

enum class Team : uint8_t {
    BLUE = 0,
    YELLOW,
};

enum class SegmentPhase : uint8_t {
    IDLE = 0,
    TURNING,
    DRIVING,
    WAITING_POINT,
};

void motionInit();
void motionTick(uint32_t now_us);
MotionState motionGetState();
Team motionGetTeam();
SegmentPhase motionGetSegmentPhase();

#endif // MOTION_H

