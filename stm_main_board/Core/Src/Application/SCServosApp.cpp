 #include "Application/SCServosApp.h"

#include "Config/Config.h"
#include "Util/logging.h"

#define UNIT_TO_DEG 0.26
#define DEG_TO_UNIT 3.79

namespace devices
{
    namespace scs_servos {

        uint8_t ids_servos[N_SERVOS] = {ID_SERVO_ARM_END_LEFT, ID_SERVO_ARM_END_RIGHT, ID_SERVO_ARM, ID_SERVO_Y_LEFT, ID_SERVO_Y_RIGHT, ID_SERVO_BANNER};
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

        /**
         * @brief Performs a homing procedure by moving until a mechanical stall is detected. To be used ONLY with servos in free rotation (i.e. rack and pinion)
         * @param ID The Servo ID.
         * @param speed The rotation speed (positive for CW, negative for CCW).
         * @param timeoutMs How long (in ms) the position must remain static to confirm stall.
         * @return true if homing is successful, false if hardware error.
         */
        bool homingByStall(uint8_t ID, int speed, uint16_t timeoutMs)
        {
            constexpr int STALL_THRESHOLD = 3;     // Minimum movement to be considered "moving"
            constexpr int POLL_INTERVAL_MS = 20;   // Polling frequency

            // 1. Ensure torque is enabled
            servos.EnableTorque(ID, 1);

            // 2. Start moving (Rotation Mode)
            // With SCS15, Wheel Mode is triggered by setting Speed and having Angle Limits set to 0.
            servos.WriteSpeed(ID, speed);

            int lastPos = servos.ReadPos(ID);
            if (lastPos == -1) return false; // Initial read failed

            uint32_t stallCounter = 0;

            while (stallCounter < timeoutMs)
            {
                osDelay(POLL_INTERVAL_MS);

                int currentPos = servos.ReadPos(ID);
                if (currentPos == -1) continue; // Skip failed UART reads

                // Calculate absolute difference
                // Note: The SCS15 encoder is 10-bit (0-1023)
                int diff = abs(currentPos - lastPos);

                // Handle the wrap-around case (e.g., jump from 1020 to 5)
                if (diff > 512)
                {
                    diff = 1024 - diff;
                }

                if (diff >= STALL_THRESHOLD)
                {
                    // Servo is still moving
                    stallCounter = 0;
                    lastPos = currentPos;
                }
                else
                {
                    // Position is stagnant, increment stall timer
                    stallCounter += POLL_INTERVAL_MS;
                }
            }

            // 3. Stall confirmed: Stop the motor immediately
            servos.WriteSpeed(ID, 0);

            return true;
        }


    }
}

using namespace devices::scs_servos;

int SCServosApp_Init()
{
    LOG_INFO("scs", "Initializing servos... (blocking until all servos are found)");
    servos = SCServos(&huart10);
    //find_ids(0, 16);
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
