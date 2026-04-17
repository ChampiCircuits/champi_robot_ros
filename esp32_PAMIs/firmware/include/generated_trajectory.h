#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 1
const float GLOBAL_SPEED_MM_S = 200.0;
const float START_AFTER_DELAY_S = 5.0;
const int TRAJECTORY_POINTS_COUNT = 3;

struct Waypoint {
    float x;
    float y;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {0.00, 0.00},
    {1000.00, 0.00},
    {0.00, 1000.00},
};

#endif // GENERATED_TRAJECTORY_H
