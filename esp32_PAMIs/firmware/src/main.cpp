#include <Arduino.h>

#include "motion.h"
#include "logging.h"

void setup() {
    Serial.begin(115200);
    sleep(1);
    Serial.println("");
    LOG_WARN("Main", "PAMI Booting...");
    LOG_WARN("Main", "PAMI Booting...");
    LOG_WARN("Main", "PAMI Booting...");
    motionInit();
}

void loop() {
    motionTick(micros());
}
