#include <Arduino.h>

#include "inputs.h"
#include "config.h"

using namespace Config;

void inputsInit() {
    pinMode(TEAM_SWITCH_PIN, TEAM_SWITCH_PULLUP ? INPUT_PULLUP : INPUT);
    // TIRETTE_PIN is analog — no explicit pinMode required.
}

Team inputsReadTeam() {
    const int level = digitalRead(TEAM_SWITCH_PIN);
    const bool active = TEAM_SWITCH_PULLUP ? (level == LOW) : (level == HIGH);
    return active ? Team::YELLOW : Team::BLUE;
}

bool inputsTiretteIsActive() {
    return analogRead(TIRETTE_PIN) > 512;
}
