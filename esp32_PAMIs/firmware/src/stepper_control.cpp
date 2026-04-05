#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "stepper_control.h"

namespace {

using namespace Config;

struct StepperAxis {
    int step_pin;
    int dir_pin;
    bool invert_dir;
    float steps_per_mm;
    float target_steps_s;
    uint32_t half_period_us;
    uint32_t next_toggle_us;
    bool step_level;
};

StepperAxis g_left_axis{LEFT_STEP_PIN, LEFT_DIR_PIN, false, 0.0f, 0.0f, 0, 0, false};
StepperAxis g_right_axis{RIGHT_STEP_PIN, RIGHT_DIR_PIN, true, 0.0f, 0.0f, 0, 0, false};

bool timeReachedUs(uint32_t now_us, uint32_t deadline_us) {
    return static_cast<int32_t>(now_us - deadline_us) >= 0;
}

uint32_t periodFromStepsPerSecond(float abs_steps_s) {
    if (abs_steps_s < MIN_ACTIVE_STEPS_S) {
        return 0;
    }
    const float half_period_us = 500000.0f / abs_steps_s;
    return static_cast<uint32_t>(half_period_us);
}

void stepperSetSpeedMmS(StepperAxis &axis, float speed_mm_s, uint32_t now_us) {
    const float steps_s = speed_mm_s * axis.steps_per_mm;
    const bool forward = steps_s >= 0.0f;
    const bool dir_level = axis.invert_dir ? !forward : forward;
    digitalWrite(axis.dir_pin, dir_level ? HIGH : LOW);

    axis.target_steps_s = steps_s;
    axis.half_period_us = periodFromStepsPerSecond(fabsf(steps_s));
    if (axis.half_period_us == 0) {
        axis.step_level = false;
        digitalWrite(axis.step_pin, LOW);
        axis.next_toggle_us = 0;
    } else if (axis.next_toggle_us == 0) {
        axis.next_toggle_us = now_us + axis.half_period_us;
    }
}

void tickAxis(StepperAxis &axis, uint32_t now_us) {
    if (axis.half_period_us == 0 || axis.next_toggle_us == 0) {
        return;
    }
    if (!timeReachedUs(now_us, axis.next_toggle_us)) {
        return;
    }

    axis.step_level = !axis.step_level;
    digitalWrite(axis.step_pin, axis.step_level ? HIGH : LOW);
    axis.next_toggle_us = now_us + axis.half_period_us;
}

} // namespace

void stepperControlInit() {
    pinMode(g_left_axis.step_pin, OUTPUT);
    pinMode(g_left_axis.dir_pin, OUTPUT);
    pinMode(g_right_axis.step_pin, OUTPUT);
    pinMode(g_right_axis.dir_pin, OUTPUT);

    const float wheel_circumference_mm = static_cast<float>(PI) * WHEEL_DIAMETER_MM;
    const float steps_per_wheel_turn = MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
    const float steps_per_mm = steps_per_wheel_turn / wheel_circumference_mm;
    g_left_axis.steps_per_mm = steps_per_mm;
    g_right_axis.steps_per_mm = steps_per_mm;

    stepperControlStop();
}

void stepperControlCommandMmS(float left_mm_s, float right_mm_s, uint32_t now_us) {
    stepperSetSpeedMmS(g_left_axis, left_mm_s, now_us);
    stepperSetSpeedMmS(g_right_axis, right_mm_s, now_us);
}

void stepperControlTick(uint32_t now_us) {
    tickAxis(g_left_axis, now_us);
    tickAxis(g_right_axis, now_us);
}

void stepperControlStop() {
    g_left_axis.target_steps_s = 0.0f;
    g_right_axis.target_steps_s = 0.0f;
    g_left_axis.half_period_us = 0;
    g_right_axis.half_period_us = 0;
    g_left_axis.next_toggle_us = 0;
    g_right_axis.next_toggle_us = 0;
    g_left_axis.step_level = false;
    g_right_axis.step_level = false;
    digitalWrite(g_left_axis.step_pin, LOW);
    digitalWrite(g_right_axis.step_pin, LOW);
}

