#include <Arduino.h>

#include "motion.h"

void setup() {
    Serial.begin(115200);
    Serial.println("PAMI Booting...");
    motionInit();
}

void loop() {
    motionTick(micros());
}
