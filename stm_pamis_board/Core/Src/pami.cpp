#include "pami.h"

#include "Config/Config.h"
#include "Util/logging.h"
#include "pami_paths.h"
#include "tim.h"
#include "usart.h"

SCServos servos;
LaserSensor sensor_obstacle;
LaserSensor sensor_void;

void config_servos()
{
    servos.EnableTorque(ID_SERVO_TRACTION, 1);
    HAL_Delay(100);
    servos.EnableTorque(ID_SERVO_DIR, 1);
    HAL_Delay(100);
    servos.WriteLimitTroque(ID_SERVO_TRACTION, 1023);
    HAL_Delay(100);
    servos.WriteLimitTroque(ID_SERVO_DIR, 600);
    HAL_Delay(100);
    servos.WriteLimitVoltageMax(ID_SERVO_TRACTION, 95);
    HAL_Delay(100);
    servos.WriteLimitVoltageMax(ID_SERVO_DIR, 95);
    HAL_Delay(100);
    servos.WriteLimitAngle(ID_SERVO_TRACTION, 0, 0);
    HAL_Delay(100);
    servos.WriteLimitAngle(ID_SERVO_DIR, 0, 270);
    HAL_Delay(100);
}

bool check_servos()
{
    return true; // TODO
    int servos_ids[] = {ID_SERVO_DIR, ID_SERVO_TRACTION};

    for (auto servo_id : servos_ids)
    {
        int pos = servos.ReadPos(servo_id);
        if (pos == -1)
        {
            LOG_ERROR("init", "Servo %d not found !", servo_id);
            return false;
        }
    }

    LOG_INFO("init", "Servos test OK");
    return true;
}

void visual_check_movement()
{
    servos.WriteSpe(ID_SERVO_TRACTION, 511);
    HAL_Delay(100);
    servos.WritePos(ID_SERVO_DIR, DIR_ANGLE_LEFT____, 1000);
    HAL_Delay(1000);
    servos.WritePos(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 1000);
    HAL_Delay(3000);

    servos.WritePos(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 1000);
    HAL_Delay(100);
    servos.WriteSpe(ID_SERVO_TRACTION, 0);
    HAL_Delay(100);
}

void PAMI_Init()
{
    change_directions_according_to_color();
    check_path_duration();

    //////////////////////////////////////////////////
    // SERVOS
    //////////////////////////////////////////////////
    servos = SCServos(&huart1);
    // servos.scan_ids(0, 20);

    config_servos();
    while (!check_servos())
    {
        HAL_Delay(500);
    }

    //////////////////////////////////////////////////
    // SERVO BLEU
    //////////////////////////////////////////////////
    LOG_INFO("init", "Servo bleu test");
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    int angle = 0;  //5°
    int pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
    HAL_Delay(2000);

    angle = 180;  //5°
    pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
    HAL_Delay(2000);
    LOG_INFO("init", "Servo bleu test terminé");


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

    LOG_WARN("init", "PAMI READY !!")
}

void wait85s() { HAL_Delay(1000 * 5); } // TODO 85

void stopMotorsAndWaitForeverWithActuators()
{
    LOG_INFO("init", "End of match, waiting forever...");
    servos.WriteSpe(ID_SERVO_TRACTION, 0);
    while (true)
    {
        // we move the actuator back and forth
        int angle = 0;  //5°
        int pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
        HAL_Delay(2000);

        angle = 180;  //5°
        pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
        HAL_Delay(2000);
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
    LOG_INFO("pami", "PAMI STARTING !!!");

    // execute the path
    for (int i = 0; i < path_length; i++)
    {
        checkTimeLeft();

        Segment segment = path[i];
        LOG_INFO("pami", "new segment");

        // turn the DIR wheel
        servos.set_angle(ID_SERVO_DIR, segment.dir_servo_angle, 200);
        HAL_Delay(200); // TODO check that's enough time, and adjust in check time

        // set TRACTION velocity
        // servos.WriteSpe(ID_SERVO_TRACTION, segment.speed);
        servos.WriteSpe(ID_SERVO_TRACTION, 300);

        // wait for duration
        uint32_t segment_start_time = HAL_GetTick(); // ms
        uint32_t segment_wait_time_ms = static_cast<uint32_t>(segment.duration_s * 1000.0);

        while (HAL_GetTick() - segment_start_time < segment_wait_time_ms)
        {
            // CheckObstacle(); // TODO
            HAL_Delay(200);
        }
    }

#ifdef PAMI_SUPERSTAR
    superstar_forward_till_void();
#endif

        stopMotorsAndWaitForeverWithActuators();
}
