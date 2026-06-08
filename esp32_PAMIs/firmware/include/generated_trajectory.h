#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI 3
const float GLOBAL_SPEED_MM_S = 750.0;
const float ANGULAR_SPEED_RAD_S = 1.3962634015954636;
const float DELAY_AFTER_PULL_CORD_S = 0.0;
const int TRAJECTORY_POINTS_COUNT = 3;

struct Waypoint {
    float x;
    float y;
    float waitS;
    float headingDeg;
};

const Waypoint EXPERIMENT_TRAJECTORY[] = {
    {500.00, 0.00, 0.00, 0.0},
    {500.00, 500.00, 0.00, 0.0},
};

#endif // GENERATED_TRAJECTORY_H
