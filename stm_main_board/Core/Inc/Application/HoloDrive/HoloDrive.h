
#ifndef INC_APPLICATION_HOLODRIVE_H_
#define INC_APPLICATION_HOLODRIVE_H_


#include <SpeedStepper.h>
#include "Application/Modbus/DataStructures.h"


class HoloDrive {
public:
    HoloDrive();
    HoloDrive(const StepperTimer& stepper0, const StepperTimer& stepper1, const StepperTimer& stepper2);
    virtual ~HoloDrive();
    void set_config(HoloDriveConfig config);
    bool is_configured();
    void set_cmd_vel(Vector3 cmd);
    void write_wheels_speeds(double *speeds_rps);
    void compute_wheels_speeds(Vector3 cmd, double *ret_speeds_rps);
    void spin_once_motors_control();
    Vector3 get_current_vel();
    void update_current_vel(const double *speeds_rps);
    Vector3 compute_limited_speed();
private:
    StepperTimer steppers[3];
    Vector3 cmd_vel{};
    double current_wheels_speeds_rps[3]{};
    Vector3 current_vel;

    /*
     * max_accel_per_cycle, en rotation par seconde par cycle, est la vitesse maximale autorisée
     * ajoutable à la vitesse actuelle d'une roue à chaque cycle. Soit : Combien peut-on ajouter de vitesse
     * à une roue à chaque cycle de contrôle ?
     */
    double max_accel_per_cycle{};
    bool has_config{};

    HoloDriveConfig config_{};
};


#endif /* INC_APPLICATION_HOLODRIVE_H_ */
