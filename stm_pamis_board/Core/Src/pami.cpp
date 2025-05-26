#include "pami.h"

#include "usart.h"
#include "Devices/SCServos.h"
#include "Config/Config.h"
#include "Util/logging.h"
#include "pami_paths.h"

SCServos servos;



void config_servos() {
  servos.EnableTorque(ID_SERVO_TRACTION, 1);
  servos.EnableTorque(ID_SERVO_DIR, 1);
  HAL_Delay(100);
  servos.WriteLimitTroque(ID_SERVO_TRACTION, 1023);
  servos.WriteLimitTroque(ID_SERVO_DIR, 600);
  HAL_Delay(100);
  servos.WriteLimitVoltageMax(ID_SERVO_TRACTION, 95);
  servos.WriteLimitVoltageMax(ID_SERVO_DIR, 95);
  HAL_Delay(100);
  servos.WriteLimitAngle(ID_SERVO_TRACTION, 0, 0);
  servos.WriteLimitAngle(ID_SERVO_DIR, 0, 270);
  HAL_Delay(100);
}

bool check_servos() {
  int pos_servo_dir = servos.ReadPos(ID_SERVO_DIR);
  int pos_servo_traction = servos.ReadPos(ID_SERVO_TRACTION);
  if (pos_servo_dir == -1 || pos_servo_traction == -1) {
    LOG_ERROR("pami", "Error reading servos. servo_dir=%d, servo_traction=%d", pos_servo_dir, pos_servo_traction);
    return false;
  }
  LOG_INFO("pami", "Servos test OK");
  return true;
}

void visual_check_movement() {
  servos.WriteSpe(ID_SERVO_TRACTION, 511);

  servos.WritePos(ID_SERVO_DIR, 0, 1000);
  HAL_Delay(1000);
  servos.WritePos(ID_SERVO_DIR, 800, 1000);
  HAL_Delay(1000);


  servos.WritePos(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 1000);
  servos.WriteSpe(ID_SERVO_TRACTION, 0);
}


void PAMI_Init() {
  change_directions_according_to_color();
  check_path_duration();

  servos = SCServos(&huart1);

  servos.scan_ids(0, 20);

  while (!check_servos()) {
    HAL_Delay(2000);
  }

  config_servos(); // TODO dans quel ordre ?
}

void wait85s() {
  HAL_Delay(1000*85);
}

void CheckObstacle() {
  // TODOOOOOOOO
}

void PAMI_Loop() {
  wait85s();

  // execute the path
  for (int i=0; i < path_length; i++)
  {
    Segment current_segment = path[i];

    // turn the DIR wheel
    servos.set_angle(ID_SERVO_DIR, current_segment.dir_servo_angle, 200);
    HAL_Delay(200); // TODO check that's enough time, and adjust in check time

    // set TRACTION velocity
    servos.WriteSpe(ID_SERVO_TRACTION, current_segment.speed);

    // wait for duration
    uint32_t start_time = HAL_GetTick(); // temps en ms
    uint32_t wait_time_ms = static_cast<uint32_t>(current_segment.duration_s / 1000.0);

    while (HAL_GetTick() - start_time < wait_time_ms) {
      CheckObstacle();

      HAL_Delay(200);
    }

  }

}
