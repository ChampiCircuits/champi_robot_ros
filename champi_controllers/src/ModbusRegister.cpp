
#include "champi_controllers/ModbusRegister.h"

namespace mod_reg
{
    uint16_t registers[REGISTERS_SIZE];

    // EDIT HERE BEGIN

    // Declare pointers to structures of data we want to send/receive
    Vector3* cmd_vel;
    Vector3* measured_vel;
    BaseConfig* base_config;

    // Metadata to gather info needed to read/write the data (for master only)
    register_metadata reg_cmd_vel;
    register_metadata reg_measured_vel;
    register_metadata reg_base_config;

    // EDIT HERE END

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

    void init_register_metadata(register_metadata &reg_meta, uint16_t* ptr, uint16_t size)
    {
        reg_meta.ptr = ptr;
        reg_meta.size = size;
        reg_meta.address = ptr - registers;
    }

    void setup_registers_slave(){

        // EDIT HERE BEGIN

        cmd_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
        measured_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
        base_config = (BaseConfig*) init_ptr_to_register( sizeof(BaseConfig) / sizeof(uint16_t));

        // EDIT HERE END
    }

    void setup_registers_master(){

        setup_registers_slave();

        // EDIT HERE BEGIN

        init_register_metadata(reg_cmd_vel, (uint16_t*) cmd_vel, sizeof(Vector3) / sizeof(uint16_t));
        init_register_metadata(reg_measured_vel, (uint16_t*) measured_vel, sizeof(Vector3) / sizeof(uint16_t));
        init_register_metadata(reg_base_config, (uint16_t*) base_config, sizeof(BaseConfig) / sizeof(uint16_t));

        // EDIT HERE END
    }
}

