#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

struct Vector3
{
    double x;
    double y;
    double theta;
};

struct HoloDriveConfig
{
    double wheel_radius;
    double base_radius;
    double max_speed_linear;
    double max_accel_wheel;
    double max_accel_linear;
    double max_decel_linear;
    double max_accel_angular;
    double max_decel_angular;
};

struct StmConfig
{
    bool is_set;
    double cmd_vel_timeout;
    HoloDriveConfig holo_drive_config;
};

#endif //DATASTRUCTURES_H
