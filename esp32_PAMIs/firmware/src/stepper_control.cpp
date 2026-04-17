#include <Arduino.h>

#include "config.h"
#include "stepper_control.h"
#include "logging.h"

namespace {

using namespace Config;

// ESP32 LEDC channels for hardware PWM step generation
constexpr uint8_t LEFT_LEDC_CHANNEL = 0;
constexpr uint8_t RIGHT_LEDC_CHANNEL = 1;
constexpr uint8_t LEDC_RESOLUTION_BITS = 8;
constexpr uint32_t LEDC_DUTY_50_PERCENT = 128;  // 50% of 256
constexpr uint32_t MIN_STEP_FREQ_HZ = 20;

struct SpeedStepper {
    int step_pin;
    int dir_pin;
    uint8_t ledc_channel;
    bool invert_dir;
    float steps_per_mm;
    float current_speed_mm_s;
    bool stopped;
};

SpeedStepper g_left_stepper{LEFT_STEP_PIN, LEFT_DIR_PIN, LEFT_LEDC_CHANNEL, false, 0.0f, 0.0f, true};
SpeedStepper g_right_stepper{RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_LEDC_CHANNEL, true, 0.0f, 0.0f, true};

void stepperSetSpeedHz(SpeedStepper &stepper, int hz, bool forward) {
    const bool dir_level = stepper.invert_dir ? !forward : forward;
    digitalWrite(stepper.dir_pin, dir_level ? HIGH : LOW);

    if (hz < static_cast<int>(MIN_STEP_FREQ_HZ)) {
        if (!stepper.stopped) {
            ledcWrite(stepper.ledc_channel, 0);  // Stop PWM
            stepper.stopped = true;
        }
        return;
    }

    if (stepper.stopped) {
        stepper.stopped = false;
    }

    // Change frequency and set 50% duty cycle
    ledcSetup(stepper.ledc_channel, hz, LEDC_RESOLUTION_BITS);
    ledcWrite(stepper.ledc_channel, LEDC_DUTY_50_PERCENT);
}

void stepperSetSpeedMmS(SpeedStepper &stepper, float speed_mm_s) {
    if (stepper.current_speed_mm_s == speed_mm_s) {
        return;
    }

    digitalWrite(ENABLE_MOTORS, LOW);  // Enable drivers

    const float steps_s = speed_mm_s * stepper.steps_per_mm;
    const int hz = static_cast<int>(fabsf(steps_s));
    const bool forward = steps_s >= 0.0f;

    stepperSetSpeedHz(stepper, hz, forward);
    stepper.current_speed_mm_s = speed_mm_s;
}

void stepperStop(SpeedStepper &stepper) {
    ledcWrite(stepper.ledc_channel, 0);
    stepper.stopped = true;
    stepper.current_speed_mm_s = 0.0f;
}

void stepperInit(SpeedStepper &stepper) {
    pinMode(stepper.dir_pin, OUTPUT);
    digitalWrite(stepper.dir_pin, LOW);

    // Setup LEDC channel and attach to step pin
    ledcSetup(stepper.ledc_channel, 1000, LEDC_RESOLUTION_BITS);
    ledcAttachPin(stepper.step_pin, stepper.ledc_channel);
    ledcWrite(stepper.ledc_channel, 0);  // Start stopped

    const float wheel_circumference_mm = static_cast<float>(PI) * WHEEL_DIAMETER_MM;
    const float steps_per_wheel_turn = MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
    stepper.steps_per_mm = steps_per_wheel_turn / wheel_circumference_mm;
    stepper.stopped = true;
    stepper.current_speed_mm_s = 0.0f;
}

}  // namespace

void stepperControlInit() {
    pinMode(ENABLE_MOTORS, OUTPUT);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Disabled at start

    stepperInit(g_left_stepper);
    stepperInit(g_right_stepper);

    LOG_INFO("Stepper", "Hardware PWM init: left=ch%d right=ch%d", LEFT_LEDC_CHANNEL, RIGHT_LEDC_CHANNEL);
}

void stepperControlCommandMmS(float left_mm_s, float right_mm_s, uint32_t now_us) {
    (void)now_us;  // Not needed with hardware PWM
    stepperSetSpeedMmS(g_left_stepper, left_mm_s);
    stepperSetSpeedMmS(g_right_stepper, right_mm_s);
}

void stepperControlTick(uint32_t now_us) {
    (void)now_us;  // Nothing to do - hardware PWM handles timing
}

void stepperControlStop() {
    stepperStop(g_left_stepper);
    stepperStop(g_right_stepper);
    // Keep ENABLE_MOTORS LOW (active) so drivers hold torque and wheels don't slip
    digitalWrite(ENABLE_MOTORS, LOW);
}




