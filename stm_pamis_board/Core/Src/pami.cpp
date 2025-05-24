#include "pami.h"

#include "usart.h"
#include "Devices/SCServos.h"
#include "Config/Config.h"
#include "Util/logging.h"

SCServos servos;

#define ID_SERVO_DIR 18
#define ID_SERVO_TRACTION 17

void PAMI_Init() {
  servos = SCServos(&huart1);

  // LOG_INFO("pami","before scanning");
  // servos.scan_ids(0, 20);
  // LOG_INFO("pami","afterscanning");



  // servos.EnableTorque(ID_SERVO_DIR, 1);
  // HAL_Delay(100);
  // servos.WriteLimitTroque(ID_SERVO_DIR, 600);
  // HAL_Delay(100);
  // servos.WriteLimitVoltageMax(ID_SERVO_DIR, 95);
  // HAL_Delay(100);

  servos.EnableTorque(ID_SERVO_TRACTION, 1);
  HAL_Delay(100);
  servos.WriteLimitTroque(ID_SERVO_TRACTION, 600);
  HAL_Delay(100);
  servos.WriteLimitVoltageMax(ID_SERVO_TRACTION, 95);
  HAL_Delay(100);
  servos.WriteLimitAngle(ID_SERVO_TRACTION, 0, 0);
  HAL_Delay(100);

  // read pose DIR servo
  int pos = servos.ReadPos(ID_SERVO_DIR);
  if (pos == -1) {
    LOG_ERROR("pami", "Servo DIR not found");
  } else {
    LOG_INFO("pami", "Servo DIR position: %d", pos);
  }
 
  // test turn back and forth
  // while (1) {
  //   servos.WritePos(ID_SERVO_DIR, 200, 1000);
  //   HAL_Delay(1000);
  //   servos.WritePos(ID_SERVO_DIR, 800, 1000);
  //   HAL_Delay(1000);
  // }

  // test spin
  servos.WriteSpe(ID_SERVO_TRACTION, 511);

}

void PAMI_Loop() {

  HAL_Delay(1000);

}
