#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 2
const float GLOBAL_SPEED_MM_S = 350.0;
const float ANGULAR_SPEED_RAD_S = 1.2217304763960306;
const float DELAY_AFTER_PULL_CORD_S = 1.0;
const int TRAJECTORY_POINTS_COUNT = 4;

struct Waypoint {
    float x;
    float y;
    float waitS;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {250.00, 1750.00, 0.50},
    {500.00, 1750.00, 0.00},
    {500.00, 1250.00, 0.00},
    {1450.00, 850.00, 0.00},
};

#endif // GENERATED_TRAJECTORY_H
