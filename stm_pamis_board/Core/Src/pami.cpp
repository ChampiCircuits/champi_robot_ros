#include "pami.h"

#include "Config/Config.h"
#include "Util/logging.h"
#include "pami_paths.h"
#include "usart.h"

void config_servos()
{
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

bool check_servos()
{
    int servos_ids[] = {ID_SERVO_DIR, ID_SERVO_TRACTION, ID_SERVO_ACTUATOR};

    for (auto servo_id : servos_ids)
    {
        int pos = servos.ReadPos(servo_id);
        if (pos == -1)
        {
            LOG_ERROR("pami", "Servo %d not found !", servo_id);
            return false;
        }
    }

    LOG_INFO("pami", "Servos test OK");
    return true;
}

void visual_check_movement()
{
    servos.WriteSpe(ID_SERVO_TRACTION, 511);

    servos.WritePos(ID_SERVO_DIR, 0, 1000);
    HAL_Delay(1000);
    servos.WritePos(ID_SERVO_DIR, 800, 1000);
    HAL_Delay(1000);

    servos.WritePos(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 1000);
    servos.WriteSpe(ID_SERVO_TRACTION, 0);
}

void PAMI_Init()
{
    change_directions_according_to_color();
    check_path_duration();

    //////////////////////////////////////////////////
    // SERVOS
    //////////////////////////////////////////////////
    servos = SCServos(&huart1);
    servos.scan_ids(0, 20);

    while (!check_servos())
    {
        HAL_Delay(2000);
    }

    config_servos(); // TODO dans quel ordre ?

    //////////////////////////////////////////////////
    // SENSORS
    //////////////////////////////////////////////////
    sensor_obstacle = LaserSensor(
        XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin,
        SENSOR_SENSOR_OBSTACLE_ADDRESS, SENSOR_SENSOR_OBSTACLE_OFFSET);

    // TODO checker le 100, 100 de l'IOC
    sensor_obstacle.setup();

#ifdef PAMI_SUPERSTAR
  sensor_void =
      LaserSensor(XSHUT_SENSOR_VOID_GPIO_Port, XSHUT_SENSOR_VOID_Pin,
                  SENSOR_SENSOR_VOID_ADDRESS,
                  SENSOR_SENSOR_VOID_OFFSET); // must disable other sensors to
                                              // setup a new one

  sensor_obstacle.disableSensor();
  sensor_void.setup();
  sensor_obstacle.enableSensor();
#endif
}

void wait85s() { HAL_Delay(1000 * 85); }

void stopMotorsAndWaitForeverWithActuators()
{
    servos.WriteSpe(ID_SERVO_TRACTION, 0);
    while (true)
    {
        // we move the actuator back and forth
        servos.set_angle(ID_SERVO_ACTUATOR, ACTUATOR_POSE1_ANGLE, 200);
        HAL_Delay(1000);
        servos.set_angle(ID_SERVO_ACTUATOR, ACTUATOR_POSE2_ANGLE, 200);
        HAL_Delay(1000);
    }
}

void checkTimeLeft()
{
    static uint32_t pami_start_time = HAL_GetTick(); // ms
    static uint32_t max_time_ms = 15 * 1000;

    if (HAL_GetTick() - pami_start_time > max_time_ms)
    {
        stopMotorsAndWaitForeverWithActuators();
    }
}

void CheckObstacle()
{
    while (sensor_obstacle.get_dist_mm() < STOPPING_DISTANCE_MM)
    {
        checkTimeLeft();
        HAL_Delay(200);
    }
}

void superstar_forward_till_void()
{
    servos.set_angle(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 200);
    HAL_Delay(200);
    // set TRACTION velocity
    servos.WriteSpe(ID_SERVO_TRACTION, LOW_SPEED);

    while (sensor_void.get_dist_mm() < STOPPING_DISTANCE_TO_EDGE_MM)
    {
        checkTimeLeft();
        HAL_Delay(200);
    }
    // stop motor
    servos.WriteSpe(ID_SERVO_TRACTION, 0);
}

void PAMI_Main()
{
    wait85s();

    // execute the path
    for (int i = 0; i < path_length; i++)
    {
        checkTimeLeft();

        auto [dir_servo_angle, duration_s, speed] = path[i];

        // turn the DIR wheel
        servos.set_angle(ID_SERVO_DIR, dir_servo_angle, 200);
        HAL_Delay(200); // TODO check that's enough time, and adjust in check time

        // set TRACTION velocity
        servos.WriteSpe(ID_SERVO_TRACTION, speed);

        // wait for duration
        uint32_t segment_start_time = HAL_GetTick(); // ms
        uint32_t segment_wait_time_ms = static_cast<uint32_t>(duration_s / 1000.0);

        while (HAL_GetTick() - segment_start_time < segment_wait_time_ms)
        {
            CheckObstacle();
            HAL_Delay(200);
        }

#ifdef PAMI_SUPERSTAR
    superstar_forward_till_void();
#endif

        stopMotorsAndWaitForeverWithActuators();
    }
}
