#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 6
const float GLOBAL_SPEED_MM_S = 350.0;
const float DELAY_AFTER_PULL_CORD_S = 1.0;
const int TRAJECTORY_POINTS_COUNT = 3;

struct Waypoint {
    float x;
    float y;
    float waitS;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {50.00, 1900.00, 2.00},
    {200.00, 1900.00, 0.00},
    {750.00, 850.00, 0.00},
};

#endif // GENERATED_TRAJECTORY_H
