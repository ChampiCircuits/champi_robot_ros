#include "pami.h"

#include "usart.h"
#include "Devices/SCServos.h"
#include "Config/Config.h"
#include "Util/logging.h"

void PAMI_Init() {

  devices::scs_servos::servos = SCServos(&huart1);

  LOG_INFO("pami","before scanning");
  devices::scs_servos::servos.scan_ids(0, 20);
  LOG_INFO("pami","afterscanning");

  devices::scs_servos::init_successful = false;
  while (devices::scs_servos::test() == -1) {
    LOG_ERROR("scs", "Error initializing servos. Retrying.");
    HAL_Delay(1000);
  }

  devices::scs_servos::set_enable(true); // TODO move to sysTask

  for (const auto id : devices::scs_servos::ids_servos) {
    devices::scs_servos::servos.WriteLimitTroque(id, SCSERVOS_TORQUE_LIMIT);
    HAL_Delay(1);
  }
  devices::scs_servos::init_successful = true;

}

void PAMI_Loop() {
  devices::scs_servos::set_angle(ID_SERVO_TEST, 100, 100);

  HAL_Delay(1000);
  devices::scs_servos::set_angle(ID_SERVO_TEST, 0, 100);
  HAL_Delay(1000);

}
