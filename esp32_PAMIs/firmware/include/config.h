#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include <stdint.h>

namespace Config {

// GPIO MAPPING
// switches
constexpr int TEAM_SWITCH_PIN = 26; //D26
constexpr int TIRETTE_PIN = 25;     //D25
// ultrasonic sensor
constexpr int US_TRIG_PIN = 18;  //D18 // inversed in schematic with ECHO
constexpr int US_ECHO_PIN = 5; //D5
// led rgb
constexpr bool LED_IS_COMMON_ANODE = true; // common pin is the (+)
constexpr int LED_R_PIN = 27;   //D27
constexpr int LED_G_PIN = 14;   //D14
constexpr int LED_B_PIN = 12;   //D12
// steppers
constexpr int LEFT_STEP_PIN = 4;    //D4
constexpr int LEFT_DIR_PIN = 2;     //D2
constexpr int RIGHT_STEP_PIN = 21;  //D21
constexpr int RIGHT_DIR_PIN = 19;   //D19
constexpr int ENABLE_MOTORS = 23;   //D23
// actuator (12v DC motor)
constexpr int ACTUATOR_PIN = 22;    //D22

// Input logic.
constexpr bool TEAM_SWITCH_PULLUP = true; // TODO put to false
// constexpr bool TIRETTE_PULLUP = false;
// constexpr uint32_t TIRETTE_DEBOUNCE_MS = 40;

// Map/team transform.
constexpr float MAP_WIDTH_MM = 3000.0f;

// Obstacle sensing.
constexpr float OBSTACLE_STOP_MM = 40.0f;
constexpr float OBSTACLE_RESUME_MM = 50.0f;
constexpr uint32_t US_TIMEOUT_US = 25000;
constexpr uint8_t MAX_US_INVALID_BEFORE_BLOCK = 5;

// Telemetry cadence.
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;

// Drivetrain geometry and motor setup.
constexpr float WHEEL_DIAMETER_MM = 72.0f;
constexpr float ENTRAXE_MM = 122.0f; // plus c'est haut plus il va tourner
// constexpr float ENTRAXE_MM = 133.5f;
constexpr float MOTOR_STEPS_PER_REV = 200.0f; // 1 step = 1.8° --> steps_per_rev = 360/1.8 = 200
constexpr float MICROSTEPS = 16.0f;
constexpr float GEAR_RATIO = 1.0f;

// Motion profile defaults.
constexpr float MIN_ACTIVE_STEPS_S = 1.0f;
constexpr float TURN_WHEEL_SPEED_MM_S = 80.0f;
constexpr float MIN_TURN_RAD = 0.02f;

// Accel/decel ramp profile for DRIVING phase.
constexpr float ACCEL_MM_S2 = 100.0f;  // linear acceleration [mm/s²]
constexpr float DECEL_MM_S2 = 800.0f;  // linear deceleration [mm/s²]

} // namespace MotionConfig

#endif // MOTION_CONFIG_H