#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 1
const float GLOBAL_SPEED_MM_S = 200.0;
const float DELAY_AFTER_PULL_CORD_S = 10.0;
const int TRAJECTORY_POINTS_COUNT = 5;

struct Waypoint {
    float x;
    float y;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {1677.00, 570.00},
    {1693.00, 920.00},
    {1967.00, 917.00},
    {1977.00, 1210.00},
    {1693.00, 1187.00},
};

#endif // GENERATED_TRAJECTORY_H
