
#ifndef INC_APPLICATION_HOLODRIVE_H_
#define INC_APPLICATION_HOLODRIVE_H_

#include "Application/Modbus/DataStructures.h"
#include <Devices/SpeedStepper.h>

using namespace com_types;

class HoloDrive {
public:
  HoloDrive();
  HoloDrive(const SpeedStepper &stepper_left, const SpeedStepper &stepper_right,
            const SpeedStepper &stepper_back);
  virtual ~HoloDrive();
  void set_config(HoloDriveConfig config);
  void set_cmd_vel(Vector3 cmd);
  void write_wheels_speeds(double *speeds_rps);
  void compute_wheels_speeds(Vector3 cmd, double *ret_speeds_rps);
  void spin_once_motors_control();
  Vector3 get_current_vel();
  void update_current_vel(const double *speeds_rps);
  Vector3 compute_limited_speed();

private:
  SpeedStepper steppers[3];
  Vector3 cmd_vel{};
  double current_wheels_speeds_rps[3]{};
  Vector3 current_vel{};
  HoloDriveConfig config_{};
};

#endif /* INC_APPLICATION_HOLODRIVE_H_ */
