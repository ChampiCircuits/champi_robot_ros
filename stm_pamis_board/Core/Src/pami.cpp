#include "pami.h"

#include "Config/Config.h"
#include "Util/logging.h"
#include "pami_paths.h"
#include "tim.h"
#include "usart.h"

SCServos servos;
//LaserSensor sensor_obstacle;
//LaserSensor sensor_void;

int DIR_ANGLE_STRAIGHT = 140;
int DIR_ANGLE_RIGHT___ = 170;
int DIR_ANGLE_LEFT____ = 110;

// store start time of the match
uint32_t pami_start_time = 0; // ms

void config_servos()
{
    HAL_Delay(100);
    servos.EnableTorque(ID_SERVO_DIR, 1);
    HAL_Delay(100);
    servos.WriteLimitTroque(ID_SERVO_DIR, 600);
    HAL_Delay(100);
    servos.WriteLimitVoltageMax(ID_SERVO_DIR, 95);
    HAL_Delay(100);
}

bool check_servos()
{
    //int servos_ids[] = {};
    int servos_ids[] = {ID_SERVO_DIR};

    for (auto servo_id : servos_ids)
    {
        int pos = servos.ReadPos(servo_id);
        if (pos == -1)
        {
            LOG_ERROR("init", "Servo %d not found !", servo_id);
            return false;
        }
        LOG_INFO("init", "Servo %d found !", servo_id);
    }

    LOG_INFO("init", "Servos test OK");
    return true;
}

void visual_check_movement()
{
    LOG_INFO("visual_check_movement", "Movement test");
    // servos.WriteSpe(ID_SERVO_TRACTION, 511);
    HAL_Delay(100);
    LOG_INFO("visual_check_movement", "Movement left");
    servos.set_angle(ID_SERVO_DIR, 0, 400);
    HAL_Delay(3000);
    LOG_INFO("visual_check_movement", "Movement right");
    servos.set_angle(ID_SERVO_DIR, 135, 400);
    HAL_Delay(3000);
    LOG_INFO("visual_check_movement", "Movement straight");
    servos.set_angle(ID_SERVO_DIR, 270, 400);
    HAL_Delay(3000);
    // servos.WriteSpe(ID_SERVO_TRACTION, 0);
    HAL_Delay(10000000);
}

void debug_servo_dir_positions()
{
    LOG_WARN("debug_servo_dir_positions", "Debugging servo DIR positions");
    // servos.EnableTorque(ID_SERVO_DIR, 0);
    //
    // while (1)
    // {
    //     int pos = servos.ReadPos(ID_SERVO_DIR);
    //     LOG_WARN("debug_servo_dir_positions", "Servo DIR position: %d", pos);
    //     HAL_Delay(1000);
    // }

    // while (1)
    // {
    //     servos.set_angle(ID_SERVO_DIR, 70, 400); // gauche
    //     HAL_Delay(2000);
    //     servos.set_angle(ID_SERVO_DIR, 140, 400); // tout droit
    //     HAL_Delay(2000);
    //     servos.set_angle(ID_SERVO_DIR, 200, 400); // droite
    //     HAL_Delay(2000);
    // }
}

void goForward(int speed) //[0,255]
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
}
void stop()
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

void PAMI_Init()
{
    LOG_WARN("init", "PAMI has just started ! counting down 85s...");
    pami_start_time = HAL_GetTick(); // ms

    check_path_duration();

    //////////////////////////////////////////////////
    // CHECK COLOR
    //////////////////////////////////////////////////
    change_path_according_to_color();

    //////////////////////////////////////////////////
    // LEGO MOTORS
    //////////////////////////////////////////////////

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    stop();
    //



    //////////////////////////////////////////////////
    // SERVOS
    //////////////////////////////////////////////////
    servos = SCServos(&huart1);
    HAL_Delay(100);
//     servos.scan_ids(0, 20);

    config_servos();
    while (!check_servos())
    {
        HAL_Delay(500);
    }

    debug_servo_dir_positions();

    //////////////////////////////////////////////////
    // SERVO BLEU
    //////////////////////////////////////////////////
    // LOG_INFO("init", "Servo bleu test");
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    //
    // int angle = 0;  //5°
    // int pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
    // HAL_Delay(2000);
    //
    // angle = 180;  //5°
    // pulse = 250 + (angle*5.55);  // calculate pulse value, starting from 250
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);  // TIM2->CCR1 = pulse
    // HAL_Delay(2000);
    // LOG_INFO("init", "Servo bleu test terminé");


    //////////////////////////////////////////////////
    // SENSORS
    //////////////////////////////////////////////////
//    sensor_obstacle = LaserSensor(
//        XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin,
//        SENSOR_SENSOR_OBSTACLE_ADDRESS, SENSOR_SENSOR_OBSTACLE_OFFSET);
//
//    // TODO checker le 100, 100 de l'IOC
//    sensor_obstacle.setup();

#ifdef PAMI_SUPERSTAR
//  sensor_void =
//      LaserSensor(XSHUT_SENSOR_VOID_GPIO_Port, XSHUT_SENSOR_VOID_Pin,
//                  SENSOR_SENSOR_VOID_ADDRESS,
//                  SENSOR_SENSOR_VOID_OFFSET); // must disable other sensors to
//                                              // setup a new one
//
//  sensor_obstacle.disableSensor();
//  sensor_void.setup();
//  sensor_obstacle.enableSensor();
#endif

    // visual_check_movement();
    // servos.WriteSpe(ID_SERVO_TRACTION, -511);
    // HAL_Delay(100000);
    // servos.WriteSpe(ID_SERVO_TRACTION, 0);
    // while (1) {}

    LOG_WARN("init", "PAMI READY !!")
}

void waitTill85s()
{
    LOG_INFO("wait", "waiting for the 85s start...");
//    while (HAL_GetTick() - pami_start_time < 85 * 1000)
//    {
//        // wait for 85 seconds
//        HAL_Delay(250);
//    }
}

void stopMotorsAndWaitForeverWithActuators()
{
    LOG_INFO("init", "End of match, waiting forever...");
    // servos.WriteSpe(ID_SERVO_TRACTION, 0);
    stop();
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
//    while (sensor_obstacle.get_dist_mm() < STOPPING_DISTANCE_MM)
//    {
//        checkTimeLeft();
//        HAL_Delay(200);
//    }
}

void superstar_forward_till_void()
{
    servos.set_angle(ID_SERVO_DIR, DIR_ANGLE_STRAIGHT, 200);
    HAL_Delay(200);
    // set TRACTION velocity
//    servos.WriteSpe(ID_SERVO_TRACTION, LOW_SPEED);

//    while (sensor_void.get_dist_mm() < STOPPING_DISTANCE_TO_EDGE_MM)
//    {
//        checkTimeLeft();
//        HAL_Delay(200);
//    }
    // stop motor
//    servos.WriteSpe(ID_SERVO_TRACTION, 0);
}

void PAMI_Main()
{
    LOG_WARN("main", "TIRETTE RELEASED !!!");

    waitTill85s();
    LOG_INFO("pami", "PAMI STARTING !!!");

    // execute the path
    for (int i = 0; i < path_length; i++)
    {
        checkTimeLeft();

        Segment segment = path[i];
        LOG_INFO("pami", "new segment at speed %d, with angle %d", segment.speed, segment.angle);

        // turn the DIR wheel
        servos.set_angle(ID_SERVO_DIR, segment.angle, 200);
        HAL_Delay(200); // TODO check that's enough time, and adjust in check time

        // set TRACTION velocity
        // servos.WriteSpe(ID_SERVO_TRACTION, segment.speed);
        goForward(segment.speed);

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
