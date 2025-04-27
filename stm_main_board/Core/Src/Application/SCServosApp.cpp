#include "Application/SCServosApp.h"

#include "cmsis_os2.h"

#include "Config/Config.h"
#include "Util/logging.h"

#define UNIT_TO_DEG 0.26
#define DEG_TO_UNIT 3.79

namespace devices
{
    namespace scs_servos {

        std::vector<uint8_t> ids_servos = std::vector<uint8_t>();
        SCServos servos;
        bool init_successful = false;

        int test() {
            int result = 0;
            for (const auto id : ids_servos) {
                if (servos.ReadPos(ids_servos[id]) == -1) {
                    LOG_ERROR("act", "Error reading servo number %d", ids_servos[id]);
                    result = -1;
                } else {
                    LOG_INFO("act", "Servo number %d read succesful", ids_servos[id]);
                }
            }
            return result;
        }

        void set_enable(bool enable) {
            for (const auto id : ids_servos) {
                servos.EnableTorque(id, enable);
            }
        }

        void set_angle(uint8_t id, float angle, int ms) {
            set_angle_async(id, angle, ms);
            osDelay(ms);
        }

        void set_angle_async(uint8_t id, float angle, int ms) {
            int position = (int)(angle * DEG_TO_UNIT);
            servos.WritePos(id, position, ms);
        }


    }
}

using namespace devices::scs_servos;

int SCServosApp_Init() {
    ids_servos.push_back(ID_SERVO_ARM_END);
    ids_servos.push_back(ID_SERVO_ARM);
    ids_servos.push_back(ID_SERVO_Y_FRONT);
    ids_servos.push_back(ID_SERVO_Y_SIDE);

    servos = SCServos(&huart10);

    if (test() == -1) {
        init_successful = false;
        return -1;
    }
    init_successful = true;

    set_enable(1); // TODO move to sysTask

    for (const auto id : ids_servos) {
        servos.WriteLimitTroque(id, SCSERVOS_TORQUE_LIMIT);
    }

    return 0;
}
