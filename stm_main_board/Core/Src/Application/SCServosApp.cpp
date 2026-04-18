 #include "Application/SCServosApp.h"

#include "Actuators/BoxesSorter.h"
#include "Actuators/LiftAndClamp.h"
#include "Config/Config.h"
#include "Util/logging.h"

#define UNIT_TO_DEG 0.26
#define DEG_TO_UNIT 3.79

namespace devices
{
    namespace scs_servos {

        uint8_t ids_servos[N_SERVOS] = {
            // BoxesSorter::TOP_PUSHER_SERVO_ID,
            BoxesSorter::BOTTOM_PUSHER_SERVO_ID,
            // BoxesSorter::TRAPDOOR_SERVO_ID,
            // BoxesSorter::EXIT_RAMP_SERVO_ID,
            // LiftAndClamp::CLAMP_SERVO_ID
        };
        // uint8_t ids_servos[N_SERVOS] = {
        //     BoxesSorter::TOP_PUSHER_SERVO_ID,
        //     BoxesSorter::BOTTOM_PUSHER_SERVO_ID,
        //     BoxesSorter::TRAPDOOR_SERVO_ID,
        //     BoxesSorter::EXIT_RAMP_SERVO_ID,
        //     LiftAndClamp::CLAMP_SERVO_ID
        // };
        SCServos servos;
        bool init_successful = false;

        void find_ids(uint8_t from_id, uint8_t to_id)
        {
            for (uint8_t id=from_id; id<=to_id; id++)
            {
                if (servos.ReadPos(id) != -1)
                {
                	osDelay(100);
                    LOG_INFO("scs", "Found servo: %d, pos = %d", id, servos.ReadPos(id));
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
            set_angle_async(id, angle, ms);
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
            int pos = servos.ReadPos(id);
            if (pos == -1)
            {
                LOG_ERROR("scs", "Error reading servo number %d", id);
                return;
            }
            set_angle(id, angle, 1000);
            osDelay(1500);
            set_angle(id, pos * UNIT_TO_DEG, 1000);
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

        bool homingByEndSwitch(uint8_t ID, int speed, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
        {
            // If already on the switch, back off first
            LOG_INFO("scs", "Homing servo %d by end switch on GPIO %p pin %d", ID, GPIOx, GPIO_Pin);
            // going forward a bit
            // if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
            {
                LOG_INFO("scs", "Homing servo %d: backing off a bit...", ID);
                set_speed(ID, -speed);
                while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
                    osDelay(10);
                osDelay(1000);
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
    LOG_INFO("scs", "Initializing servos... (blocking until all servos are found)");
    servos = SCServos(&huart10);
    // find_ids(0, 16);
    //test_angle(ID_SERVO_Y_FRONT, 270);

    init_successful = false;
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

    return 0;
}
