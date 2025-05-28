#include "pami.h"

#include "i2c.h"
#include "Config/Config.h"
#include "Util/logging.h"
#include "pami_paths.h"
#include "tim.h"
#include "usart.h"

SCServos servos;
LaserSensor sensor_obstacle;
LaserSensor sensor_void;

int DIR_ANGLE_STRAIGHT = 140;
int DIR_ANGLE_RIGHT___ = 200;
int DIR_ANGLE_LEFT____ = 70;

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
}

bool check_servos()
{
    int servos_ids[] = {};
    // int servos_ids[] = {ID_SERVO_DIR};

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

void wait85s()
{
    LOG_INFO("wait", "waiting for 85s...");
    HAL_Delay(1000 * 5);
} // TODO 85

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

void PAMI_Init()
{
/*    check_path_duration();

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
    // servos.scan_ids(0, 20);

    config_servos();
    // while (!check_servos())
    // {
    //     HAL_Delay(500);
    // }

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
    */

    // sensor_obstacle = LaserSensor(
    //     XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin,
    //     SENSOR_SENSOR_OBSTACLE_ADDRESS, SENSOR_SENSOR_OBSTACLE_OFFSET);

        /* Toggle Xshut pin to reset the sensors so that their addresses can be set individually*/
        HAL_GPIO_WritePin(XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
    
        // uint8_t i2c_address = 0x52; // 0x29 est l'adresse 7 bits du VL53L4CX
        // HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, i2c_address, 3, 100);

        // if (result == HAL_OK) {
        //     printf("✅ VL53L4CX détecté à l’adresse 0x%X\n", i2c_address);
        // } else {
        //     printf("❌ VL53L4CX non détecté à l’adresse 0x%X (code erreur %d)\n", i2c_address, result);
        // }

        /* Setup the first laser sensor */
        uint16_t sensor_id;
        uint8_t status;
        auto pin = XSHUT_SENSOR_OBSTACLE_Pin;
        auto port = XSHUT_SENSOR_OBSTACLE_GPIO_Port;
        auto address = 0x52;
        printf("SENSOR_PIN: %d\n", pin);

        HAL_Delay(5);
        // set the pin to high to enable the sensor
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        HAL_Delay(5);

        // set I2C address (other unset addresses XSHUT have to be pull to low before)
        // status = VL53L4CD_SetI2CAddress(0x52, address); // 0x52 is the default address
        // if (status)
        // {
        //     printf("VL53L4CD_SetI2CAddress failed with status %u\n", status);
        //     return ;
        // }

        /* (Optional) Check if there is a VL53L4CD sensor connected */+*/8&

        printf("Checking for laser sensor at address %x\n", address);
        status = VL53L4CD_GetSensorId(address, &sensor_id);
        printf("status: %d\n", status);
        printf("laser sensor id: %d\n", sensor_id);

        if (status || (sensor_id != 0xEBAA))
        {
            printf("VL53L4CD not detected at requested address\n");
            return ;
        }
        printf("VL53L4CD detected at address %x\n", address);

        /* (Mandatory) Init VL53L4CD sensor */
        printf("Initializing laser sensor\n");
        status = VL53L4CD_SensorInit(address);
        if (status)
        {
            printf("VL53L4CD ULD Loading failed\n");
            return ;
        }

        // set the offset
        status = VL53L4CD_SetOffset(address, 0);
        if (status)
        {
            printf("VL53L4CD_SetOffset failed with status %u\n", status);
            return ;
        }

        status = VL53L4CD_StartRanging(address);
        if (status)
        {
            printf("VL53L4CD_StartRanging failed with status %u\n", status);
            return ;
        }

        printf("VL53L4CD ULD ready at address %x ready\n", address);


        // AFTER ALL SETUPS WE PULL TO HIGH THE SHUTPINS to enable the sensors
        HAL_GPIO_WritePin(XSHUT_SENSOR_OBSTACLE_GPIO_Port, XSHUT_SENSOR_OBSTACLE_Pin, GPIO_PIN_SET);

    // TODO checker le 100, 100 de l'IOC
    // sensor_obstacle.setup();

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

    // visual_check_movement();
    // servos.WriteSpe(ID_SERVO_TRACTION, -511);
    // HAL_Delay(100000);
    // servos.WriteSpe(ID_SERVO_TRACTION, 0);
    // while (1) {}

    LOG_WARN("init", "PAMI READY !!")
}

void PAMI_Main()
{
    // wait for tirette
    while (HAL_GPIO_ReadPin(TIRETTE_GPIO_Port, TIRETTE_Pin) == GPIO_PIN_SET)
    {
        LOG_INFO_THROTTLE("main", 20, "waiting for tirette");
        HAL_Delay(200);
    }
    LOG_WARN("main", "TIRETTE RELEASED !!!");

    // wait85s();
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
            CheckObstacle();
            HAL_Delay(200);
        }
    }

#ifdef PAMI_SUPERSTAR
    superstar_forward_till_void();
#endif

        stopMotorsAndWaitForeverWithActuators();
}
