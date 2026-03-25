#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <stdint.h>

namespace MotionConfig {

// GPIO mapping (replace with board wiring).
constexpr int TEAM_SWITCH_PIN = 2;
constexpr int TIRETTE_PIN = 3;
constexpr int US_TRIG_PIN = 4;
constexpr int US_ECHO_PIN = 5;
constexpr int LED_R_PIN = 6;
constexpr int LED_G_PIN = 7;
constexpr int LED_B_PIN = 8;
constexpr int LEFT_STEP_PIN = 9;
constexpr int LEFT_DIR_PIN = 10;
constexpr int RIGHT_STEP_PIN = 20;
constexpr int RIGHT_DIR_PIN = 21;

// Input logic and debounce.
constexpr bool TEAM_SWITCH_PULLUP = true;
constexpr bool TIRETTE_PULLUP = true;
constexpr uint32_t TEAM_DEBOUNCE_MS = 40;
constexpr uint32_t TIRETTE_DEBOUNCE_MS = 40;

// Map/team transform.
constexpr float MAP_WIDTH_MM = 3000.0f;

// Obstacle sensing.
constexpr float OBSTACLE_STOP_MM = 200.0f;
constexpr float OBSTACLE_RESUME_MM = 260.0f;
constexpr uint32_t US_TIMEOUT_US = 25000;
constexpr uint8_t MAX_US_INVALID_BEFORE_BLOCK = 5;

// Telemetry cadence.
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;

// Drivetrain geometry and motor setup.
constexpr float WHEEL_DIAMETER_MM = 70.0f;
constexpr float TRACK_WIDTH_MM = 140.0f;
constexpr float MOTOR_STEPS_PER_REV = 200.0f;
constexpr float MICROSTEPS = 16.0f;
constexpr float GEAR_RATIO = 1.0f;

// Motion profile defaults.
constexpr float MIN_ACTIVE_STEPS_S = 1.0f;
constexpr float TURN_WHEEL_SPEED_MM_S = 80.0f;
constexpr float MIN_TURN_RAD = 0.02f;

} // namespace MotionConfig

#endif // MOTION_CONFIG_H

