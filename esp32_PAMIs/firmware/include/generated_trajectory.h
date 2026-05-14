#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 5
const float GLOBAL_SPEED_MM_S = 350.0;
const float ANGULAR_SPEED_RAD_S = 0.6981317007977318;
const float DELAY_AFTER_PULL_CORD_S = 0.0;
const int TRAJECTORY_POINTS_COUNT = 4;

struct Waypoint {
    float x;
    float y;
    float waitS;
    float headingDeg;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {175.00, 1900.00, 1.00, 0.0},
    {375.00, 1900.00, 0.00, 0.0},
    {475.00, 1450.00, 0.00, 0.0},
    {1425.00, 850.00, 0.00, 0.0},
};

#endif // GENERATED_TRAJECTORY_H
