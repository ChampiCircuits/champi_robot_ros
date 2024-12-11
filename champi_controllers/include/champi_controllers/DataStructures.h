#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

struct Vector3
{
    double x;
    double y;
    double theta;
};

struct BaseConfig
{
    bool is_set;
    double max_accel;
    double wheel_radius;
    double base_radius;
    double cmd_vel_timeout;
};

#endif //DATASTRUCTURES_H
