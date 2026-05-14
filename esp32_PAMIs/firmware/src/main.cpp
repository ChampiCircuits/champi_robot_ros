#include <Arduino.h>

#include "motion.h"
#include "inputs.h"
#include "led_status.h"
#include "config.h"
#include "generated_trajectory.h"
#include "logging.h"
#include "stepper_control.h"
#include "config.h"

// Mettre à false pour revenir au vrai programme du robot.
const bool TEST_CONTINUOUS_SPEED = false;

void setup() {
    pinMode(Config::ACTUATOR_PIN, OUTPUT);
    pinMode(13, OUTPUT);
    analogWrite(Config::ACTUATOR_PIN, 0);
    analogWrite(13, 0);
    Serial.begin(115200);
    delay(1000);
    Serial.println("");
    LOG_WARN("Main", "PAMI Booting...");
    LOG_WARN("Main", "PAMI Booting...");
    LOG_WARN("Main", "PAMI Booting...");

    inputsInit();
    ledStatusInit();
    motionInit();

    if (TEST_CONTINUOUS_SPEED) {
        LOG_WARN("Test", "Starting continuous speed test...");
        // 50 mm/s est une vitesse sûre qui ne devrait pas décrocher même sans rampe d'accélération
        stepperControlRunAtSpeed(50.0f, 50.0f);
    }
}

uint32_t g_last_loop_ms = 0;

void loop() {
    if (TEST_CONTINUOUS_SPEED) {
        // En mode test continu, on ne fait rien dans la boucle, FastAccelStepper gère les impulsions en arrière-plan.
        delay(10);
        return;
    }

    uint32_t now_ms = millis();
    if (now_ms - g_last_loop_ms >= 10) { // 100 Hz
        g_last_loop_ms = now_ms;
        
        motionTick(micros());
        ledStatusApply(motionGetState(), motionGetTeam());
        if (motionGetState() == MotionState::COMPLETED) {
            analogWrite(Config::ACTUATOR_PIN, 100); // TODO: enable actuator properly later
            analogWrite(13, 100); // TODO: enable actuator properly later
        }
    }
    
    // Give FastAccelStepper RMT interrupts time to process on this single-core chip
    delay(1);
}
