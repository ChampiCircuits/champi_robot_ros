#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 1
const float GLOBAL_SPEED_MM_S = 150.0;
const float ANGULAR_SPEED_RAD_S = 0.6981317007977318;
const float DELAY_AFTER_PULL_CORD_S = 10.0;
const int TRAJECTORY_POINTS_COUNT = 3;

struct Waypoint {
    float x;
    float y;
    float waitS;
    float headingDeg;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {0.00, 0.00, 0.00, 0.0},
    {1000.00, 0.00, 0.00, 0.0},
    {1000.00, 1000.00, 0.00, 0.0}
};

#endif // GENERATED_TRAJECTORY_H
