#ifdef MODBUS_MASTER
#include "champi_controllers/ModbusRegister.h"
#else
#include "Application/Modbus/ModbusRegister.h"
#endif // MODBUS_MASTER

namespace mod_reg
{
    uint16_t registers[REGISTERS_SIZE];

    // EDIT HERE BEGIN

    // Declare pointers to structures of data we want to send/receive
    Vector3* cmd_vel;
    Vector3* measured_vel;
    Vector3* otos_pose;
    StmConfig* stm_config;

#ifdef MODBUS_MASTER
    // Metadata to gather info needed to read/write the data (for master only)
    register_metadata reg_cmd_vel;
    register_metadata reg_measured_vel;
    register_metadata reg_otos_pose;
    register_metadata reg_stm_config;

    // EDIT HERE END

    void init_register_metadata(register_metadata &reg_meta, uint16_t* ptr, uint16_t size)
    {
        reg_meta.ptr = ptr;
        reg_meta.size = size;
        reg_meta.address = ptr - registers;
    }
#endif // MODBUS_MASTER

    uint16_t* init_ptr_to_register(uint16_t size)
    {
        static uint16_t offset = 0;
        if (offset + size > REGISTERS_SIZE) {
            // TODO handle error
            return nullptr;
        }
        uint16_t* ptr = registers + offset;
        offset += size;
        return ptr;
    }

    void setup_registers(){

        // EDIT HERE BEGIN

        cmd_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
        measured_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
        otos_pose = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
        stm_config = (StmConfig*) init_ptr_to_register( sizeof(StmConfig) / sizeof(uint16_t));

#ifdef MODBUS_MASTER
        init_register_metadata(reg_cmd_vel, (uint16_t*) cmd_vel, sizeof(Vector3) / sizeof(uint16_t));
        init_register_metadata(reg_measured_vel, (uint16_t*) measured_vel, sizeof(Vector3) / sizeof(uint16_t));
        init_register_metadata(reg_otos_pose, (uint16_t*) otos_pose, sizeof(Vector3) / sizeof(uint16_t));
        init_register_metadata(reg_stm_config, (uint16_t*) stm_config, sizeof(StmConfig) / sizeof(uint16_t));
#endif // MODBUS_MASTER

        // EDIT HERE END
    }

}

