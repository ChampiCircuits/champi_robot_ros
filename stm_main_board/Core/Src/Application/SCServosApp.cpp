 #include "Application/SCServosApp.h"

#include "Config/Config.h"
#include "Util/logging.h"

#include "Actuators/FourSuctionCup.h"
#include "Application/Actuators/ActuatorsTask.h"

#define UNIT_TO_DEG 0.26
#define DEG_TO_UNIT 3.79

namespace devices
{
    namespace scs_servos {

        uint8_t ids_servos[N_SERVOS] = {
            // LEFT_ARM_0_SERVO_ID,
            // LEFT_ARM_1_SERVO_ID,
            // LEFT_ARM_2_SERVO_ID,
            // LEFT_ARM_3_SERVO_ID,
            // THERMO_SERVO_ID,
            RIGHT_ARM_0_SERVO_ID,
            RIGHT_ARM_1_SERVO_ID,
            RIGHT_ARM_2_SERVO_ID,
            RIGHT_ARM_3_SERVO_ID 
        };
        SCServos servos;
        bool init_successful = false;

        void find_ids(uint8_t from_id, uint8_t to_id)
        {
            LOG_INFO("scs", "Finding servos IDs from %d to %d...", from_id, to_id);
            for (uint8_t id=from_id; id<=to_id; id++)
            {
                LOG_INFO("scs", "Test servo %d", id);
                if (servos.ReadPos(id) != -1)
                {
                	osDelay(100);
                    LOG_INFO("scs", "########### Found servo: %d, pos = %d", id, servos.ReadPos(id));
                    osDelay(100);
                }
            }
        }

        int test()
        {
            int result = 0;
            for (int i = 0; i < N_SERVOS; i++)
            {
                if (servos.ReadPos(ids_servos[i]) == -1)
                {
                    LOG_ERROR("scs", "Error reading servo number %d", ids_servos[i]);
                    result = -1;
                } else
                {
                    LOG_INFO("scs", "Servo number %d read successful", ids_servos[i]);
                }
                osDelay(10);
            }
            return result;
        }

        void set_enable(bool enable)
        {
            for (const auto id : ids_servos)
            {
                servos.EnableTorque(id, enable);
                osDelay(10);
            }
        }

        float read_angle(uint8_t id)
        {
            return static_cast<float>(servos.ReadPos(id)) * UNIT_TO_DEG;
        }

        void set_angle(uint8_t id, float angle, int ms)
        {
            set_angle_async(id, angle, 300);
            osDelay(ms);
        }

        void set_angle_async(uint8_t id, float angle, int ms)
        {
            int position = (int)(angle * DEG_TO_UNIT);
            if (position < 0)
                position = 0;
            else if (position > 1023)
                position = 1023;

            servos.WritePos(id, position, ms); // TODO test, des fois ca bouge pas !
            osDelay(10);
            servos.WritePos(id, position, ms);
            osDelay(10);
        }

        void test_angle(uint8_t id, float angle)
        {
            const int pos = servos.ReadPos(id);
            LOG_INFO("scs", "Testing servo %d: current pos = %f, target angle = %.1f", id, pos*UNIT_TO_DEG, angle);
            if (pos == -1)
            {
                LOG_ERROR("scs", "Error reading servo number %d", id);
                return;
            }
            set_angle(id, angle, 500);
            osDelay(1500);
            set_angle(id, pos * UNIT_TO_DEG, 500);
            osDelay(1500);

        }

        void set_speed(uint8_t ID, int speed)
        {
            servos.WriteSpeed(ID, speed);
        }

        int read_position_raw(uint8_t id)
        {
            return servos.ReadPos(id);
        }

        void print_position_loop(uint8_t id, int durationMs)
        {
            LOG_INFO("scs", "=== Position test for servo %d, %d ms ===", id, durationMs);
            int elapsed = 0;
            while (elapsed < durationMs)
            {
                int pos = servos.ReadPos(id);
                LOG_INFO("scs", "Servo %d pos = %d", id, pos);
                osDelay(50);
                elapsed += 50;
            }
            LOG_INFO("scs", "=== End position test ===");
        }

        void sweep_angle_test(uint8_t id)
        {
            devices::scs_servos::init_successful = true; // for testing without waiting for full init
            float currentAngle = read_angle(id);
            LOG_INFO("scs", "=== Sweep test servo %d: start angle=%.1f deg ===", id, currentAngle);
            osDelay(1000);
            float stepDeg = 5.0f;

            // Phase 1: sweep down to 0°
            while (currentAngle > 0.0f)
            {
                currentAngle -= stepDeg;
                if (currentAngle < 0.0f) currentAngle = 0.0f;
                set_angle(id, currentAngle, 100);
                osDelay(100);
                LOG_INFO("scs", "0 Servo %d: %.1f deg", id, read_angle(id));
            }

            // Phase 2: sweep up to 270°
            while (currentAngle < 270.0f)
            {
                currentAngle += stepDeg;
                if (currentAngle > 270.0f) currentAngle = 270.0f;
                set_angle(id, currentAngle, 100);
                osDelay(100);
                LOG_INFO("scs", "270 Servo %d: %.1f deg", id, read_angle(id));
            }

            LOG_INFO("scs", "=== End sweep test ===");
        }

        bool homingByEndSwitch(uint8_t ID, int speed, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool move_backward_first=false)
        {
            // If already on the switch, back off first
            LOG_INFO("scs", "Homing servo %d by end switch on GPIO %p pin %d", ID, GPIOx, GPIO_Pin);
            // going forward a bit
            // if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
            if (move_backward_first)
            {
                LOG_INFO("scs", "Homing servo %d: backing off a bit...", ID);
                set_speed(ID, -speed);
                while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
                    osDelay(10);
                osDelay(500);
                set_speed(ID, 0);
                osDelay(100);
            }

            // Move towards the end switch
            set_speed(ID, speed);

            while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != GPIO_PIN_RESET)
            {
                osDelay(10);
                LOG_INFO_THROTTLE("scs", 10, "Homing servo %d...", ID);
            }

            set_speed(ID, 0);
            LOG_INFO("scs", "Homing servo %d: end switch reached", ID);
            return true;
        }


    }
}

using namespace devices::scs_servos;

int SCServosApp_Init()
{
    init_successful = false;
    LOG_INFO("scs", "Initializing servos... (blocking until all servos are found)");
    servos = SCServos(&huart10);
    // find_ids(0, 24);
    // while (1) {}
    
    while (test() == -1)
    {
        LOG_ERROR("scs", "Error initializing servos. Retrying.");
        osDelay(1000);
    }
    
    set_enable(true); // TODO move to sysTask
    
    for (const auto id : ids_servos)
    {
        servos.WriteLimitTroque(id, SCSERVOS_TORQUE_LIMIT);
        osDelay(10);
    }
    init_successful = true;
    LOG_INFO("scs", "Initializing servos OK !");

    {
        // int a =50;
        // // test all servos one by one
        // set_angle(RIGHT_ARM_0_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(RIGHT_ARM_1_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(RIGHT_ARM_2_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(RIGHT_ARM_3_SERVO_ID, a, 300); // 
        // // while (1) {}

        // // test all servos one by one
        // set_angle(LEFT_ARM_0_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(LEFT_ARM_1_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(LEFT_ARM_2_SERVO_ID, a, 300); // 
        // osDelay(1000);
        // set_angle(LEFT_ARM_3_SERVO_ID, a, 300); // 
        // while (1) {}
    }


    // test_angle(12, 200);
    // sweep_angle_test(5);

    return 0;
}
