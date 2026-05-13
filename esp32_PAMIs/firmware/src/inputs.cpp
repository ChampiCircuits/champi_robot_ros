#include <Arduino.h>

#include "inputs.h"
#include "config.h"
#include "motion.h"
#include "logging.h"

using namespace Config;

void inputsInit() {
    pinMode(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(TIRETTE_PIN, INPUT);
}

Team inputsReadTeam() {
    const int level = digitalRead(TEAM_SWITCH_PIN);
    const bool active = TEAM_SWITCH_PULLUP ? (level == LOW) : (level == HIGH);
    return active ? Team::YELLOW : Team::BLUE;
}

bool inputsTiretteIsActive() {
    LOG_INFO_THROTTLE("Inputs", 100, "Reading tirette state: %d", analogRead(TIRETTE_PIN));
    return analogRead(TIRETTE_PIN) > 512;
}
