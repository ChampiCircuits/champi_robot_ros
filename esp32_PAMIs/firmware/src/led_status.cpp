#include <Arduino.h>
#include <stdint.h>

#include "led_status.h"
#include "config.h"
#include "logging.h"

// namespace {

using namespace Config;

namespace {

struct Rgb8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

constexpr Rgb8 COLOR_OFF{10, 10, 10};
constexpr Rgb8 COLOR_BLUE{0, 0, 255};
constexpr Rgb8 COLOR_YELLOW{255, 120, 0};
constexpr Rgb8 COLOR_ORANGE{255, 30, 0};
constexpr Rgb8 COLOR_RED{255, 0, 0};
constexpr Rgb8 COLOR_GREEN{0, 255, 0};

Rgb8 rgbForColor(const LedColor color) {
    switch (color) {
        case LedColor::OFF: return COLOR_OFF;
        case LedColor::BLUE: return COLOR_BLUE;
        case LedColor::YELLOW: return COLOR_YELLOW;
        case LedColor::ORANGE: return COLOR_ORANGE;
        case LedColor::RED: return COLOR_RED;
    case LedColor::GREEN: return COLOR_GREEN;
    }
    return COLOR_OFF;
}

} // namespace

namespace {

void setLedRgb(uint8_t r, uint8_t g, uint8_t b) {
    if (LED_IS_COMMON_ANODE) {
        r = static_cast<uint8_t>(255u - r);
        g = static_cast<uint8_t>(255u - g);
        b = static_cast<uint8_t>(255u - b);
    }
    delay(5);
    analogWrite(LED_R_PIN, r);
    analogWrite(LED_G_PIN, g);
    analogWrite(LED_B_PIN, b);
}

} // namespace

void setLedColor(const LedColor color) {
    const Rgb8 rgb = rgbForColor(color);
    setLedRgb(rgb.r, rgb.g, rgb.b);
}

// } // namespace

void ledStatusInit() {
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    setLedColor(LedColor::OFF);
}

void ledStatusApply(const MotionState state, const Team latched_team, SegmentPhase g_segment_phase) {
    if (g_segment_phase == SegmentPhase::WAITING_POINT)
    {
    if ((millis() / 1000u) % 2u == 0u) {
        if (latched_team == Team::YELLOW) {
            setLedColor(LedColor::YELLOW);
        } else {
            setLedColor(LedColor::BLUE);
        }
        } else {
        setLedColor(LedColor::OFF);
        }
        return;   
    }
    switch (state) {
        case MotionState::WAITING_TIRETTE:
            // color of team when waiting for start
            if (latched_team == Team::YELLOW) {
                setLedColor(LedColor::YELLOW);
            } else {
                setLedColor(LedColor::BLUE);
            }
            break;
        case MotionState::START_DELAY:
            if ((millis() / 1000u) % 2u == 0u) {
                if (latched_team == Team::YELLOW) {
                    setLedColor(LedColor::YELLOW);
                } else {
                    setLedColor(LedColor::BLUE);
                }
            } else {
                setLedColor(LedColor::OFF);
            }
            break;
        case MotionState::RUNNING:
            {
                setLedColor(LedColor::GREEN);
                break;
            }
        case MotionState::PAUSED_OBSTACLE:
            {
                setLedColor(LedColor::ORANGE);
                break;
            }
        case MotionState::COMPLETED:
            setLedColor(LedColor::OFF);
            break;
        case MotionState::FAULT:
            setLedColor(LedColor::RED);
            break;
    }
}

