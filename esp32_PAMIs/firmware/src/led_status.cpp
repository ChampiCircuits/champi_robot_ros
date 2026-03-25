#include <Arduino.h>

#include "led_status.h"
#include "motion_config.h"

namespace {

using namespace MotionConfig;

void setLedRgb(bool r, bool g, bool b) {
    digitalWrite(LED_R_PIN, r ? HIGH : LOW);
    digitalWrite(LED_G_PIN, g ? HIGH : LOW);
    digitalWrite(LED_B_PIN, b ? HIGH : LOW);
}

} // namespace

void ledStatusInit() {
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    setLedRgb(false, false, false);
}

void ledStatusApply(MotionState state, Team latched_team) {
    switch (state) {
        case MotionState::WAITING_TIRETTE:
            // Orange while waiting for a valid start.
            setLedRgb(true, true, false);
            break;
        case MotionState::START_DELAY:
        case MotionState::RUNNING:
        case MotionState::PAUSED_OBSTACLE:
        case MotionState::COMPLETED:
            if (latched_team == Team::YELLOW) {
                setLedRgb(true, true, false);
            } else {
                setLedRgb(false, false, true);
            }
            break;
        case MotionState::FAULT:
            setLedRgb(true, false, false);
            break;
    }
}

