#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>

#include "motion.h"

enum class LedColor : uint8_t {
	OFF = 0,
	BLUE,
	YELLOW,
	ORANGE,
	RED,
	GREEN,
};

void ledStatusInit();
void ledStatusApply(MotionState state, Team latched_team, SegmentPhase g_segment_phase);
void setLedColor(LedColor color);

#endif // LED_STATUS_H

