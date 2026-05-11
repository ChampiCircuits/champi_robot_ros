#include <Arduino.h>
#include "FastAccelStepper.h"

#include "config.h"
#include "stepper_control.h"
#include "logging.h"

namespace {

using namespace Config;

FastAccelStepperEngine g_engine = FastAccelStepperEngine();
FastAccelStepper *g_left = nullptr;
FastAccelStepper *g_right = nullptr;

float g_steps_per_mm = 0.0f;
int32_t g_start_left = 0;
int32_t g_start_right = 0;
int32_t g_target_left = 0;
int32_t g_target_right = 0;

}  // namespace

void stepperControlInit() {
    pinMode(ENABLE_MOTORS, OUTPUT);
    digitalWrite(ENABLE_MOTORS, HIGH);  // Disabled at start

    g_engine.init();

    g_left = g_engine.stepperConnectToPin(LEFT_STEP_PIN);
    if (g_left) {
        g_left->setDirectionPin(LEFT_DIR_PIN, true); // HIGH = forward
        // Nous gérons le pin ENABLE_MOTORS manuellement avec digitalWrite()
        // pour éviter les conflits puisque les deux moteurs partagent cette même broche.
    } else {
        LOG_ERROR("Stepper", "Failed to init left stepper");
    }

    g_right = g_engine.stepperConnectToPin(RIGHT_STEP_PIN);
    if (g_right) {
        g_right->setDirectionPin(RIGHT_DIR_PIN, false); // LOW = forward (inverted mechanically)
        // Pas de setEnablePin ni de setAutoEnable pour la même raison.
    } else {
        LOG_ERROR("Stepper", "Failed to init right stepper");
    }

    const float wheel_circumference_mm = static_cast<float>(PI) * WHEEL_DIAMETER_MM;
    const float steps_per_wheel_turn = MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
    g_steps_per_mm = steps_per_wheel_turn / wheel_circumference_mm;

    LOG_INFO("Stepper", "FastAccelStepper HW init. steps_per_mm=%.2f", g_steps_per_mm);
}

void stepperControlSetProfile(float speed_mm_s, float accel_mm_s2, float decel_mm_s2) {
    if (!g_left || !g_right) return;

    uint32_t speed_hz = static_cast<uint32_t>(fabsf(speed_mm_s * g_steps_per_mm));
    if (speed_hz == 0) speed_hz = 1;
    uint32_t accel_hz2 = static_cast<uint32_t>(fabsf(accel_mm_s2 * g_steps_per_mm));
    if (accel_hz2 == 0) accel_hz2 = 1;
    // FastAccelStepper doesn't natively support separate accel/decel yet (without using absolute jump APIs),
    // but its deceleration will default to whatever the acceleration is set to.
    // For simplicity, we just use the accel value.

    g_left->setSpeedInHz(speed_hz);
    g_left->setAcceleration(accel_hz2);
    
    g_right->setSpeedInHz(speed_hz);
    g_right->setAcceleration(accel_hz2);
}

void stepperControlDrive(float distance_mm) {
    if (!g_left || !g_right) return;
    
    digitalWrite(ENABLE_MOTORS, LOW); // Enable hold torque

    g_start_left = g_left->getCurrentPosition();
    g_start_right = g_right->getCurrentPosition();

    int32_t steps = static_cast<int32_t>(roundf(distance_mm * g_steps_per_mm));
    
    g_target_left = g_start_left + steps;
    g_target_right = g_start_right + steps; // right is mechanically inverted but Dir pin is inverted, so same sign

    g_left->moveTo(g_target_left);
    g_right->moveTo(g_target_right);
}

void stepperControlTurn(float angle_rad) {
    if (!g_left || !g_right) return;

    digitalWrite(ENABLE_MOTORS, LOW); // Enable hold torque

    g_start_left = g_left->getCurrentPosition();
    g_start_right = g_right->getCurrentPosition();

    float arc_mm = angle_rad * ENTRAXE_MM / 2.0f;
    int32_t steps = static_cast<int32_t>(roundf(arc_mm * g_steps_per_mm));

    g_target_left = g_start_left - steps;
    g_target_right = g_start_right + steps;

    g_left->moveTo(g_target_left);
    g_right->moveTo(g_target_right);
}

void stepperControlDecelerateStop() {
    if (g_left) g_left->stopMove();
    if (g_right) g_right->stopMove();
    digitalWrite(ENABLE_MOTORS, LOW); // Hold torque
}

void stepperControlDisable() {
    if (g_left) g_left->forceStop();
    if (g_right) g_right->forceStop();
    digitalWrite(ENABLE_MOTORS, HIGH); // Disable hold torque
}

void stepperControlResume() {
    if (!g_left || !g_right) return;
    digitalWrite(ENABLE_MOTORS, LOW);
    g_left->moveTo(g_target_left);
    g_right->moveTo(g_target_right);
}

bool stepperControlIsRunning() {
    bool left_running = g_left && g_left->isRunning();
    bool right_running = g_right && g_right->isRunning();
    return left_running || right_running;
}

float stepperControlTraveledMm() {
    if (!g_left) return 0.0f;
    int32_t current = g_left->getCurrentPosition();
    int32_t diff = current - g_start_left;
    return static_cast<float>(abs(diff)) / g_steps_per_mm;
}

void stepperControlRunAtSpeed(float left_mm_s, float right_mm_s) {
    if (!g_left || !g_right) return;
    
    digitalWrite(ENABLE_MOTORS, LOW);
    
    uint32_t l_hz = static_cast<uint32_t>(fabsf(left_mm_s * g_steps_per_mm));
    uint32_t r_hz = static_cast<uint32_t>(fabsf(right_mm_s * g_steps_per_mm));
    
    if (l_hz > 0) {
        g_left->setSpeedInHz(l_hz);
        g_left->setAcceleration(10000); // Set high accel for runAtSpeed
        if (left_mm_s > 0) {
            g_left->runForward();
        } else {
            g_left->runBackward();
        }
    } else {
        g_left->stopMove();
    }
    
    if (r_hz > 0) {
        g_right->setSpeedInHz(r_hz);
        g_right->setAcceleration(10000); // Set high accel
        if (right_mm_s > 0) {
            g_right->runForward();
        } else {
            g_right->runBackward();
        }
    } else {
        g_right->stopMove();
    }
}
