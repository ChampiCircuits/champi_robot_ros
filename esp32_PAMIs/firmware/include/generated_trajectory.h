#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 1
const float GLOBAL_SPEED_MM_S = 200.0;
const float START_AFTER_DELAY_S = 5.0;
const int TRAJECTORY_POINTS_COUNT = 5;

struct Waypoint {
    float x;
    float y;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {550.00, 710.00},
    {927.00, 1060.00},
    {1370.00, 1400.00},
    {2057.00, 1437.00},
    {2340.00, 1110.00},
};

#endif // GENERATED_TRAJECTORY_H
