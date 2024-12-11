#ifndef MODBUS_REGISTERS_H
#define MODBUS_REGISTERS_H

//#include <sys/_stdint.h>
#include <stdint.h>
#include "DataStructures.h"

#define REGISTERS_SIZE 500

namespace mod_reg
{

    struct register_metadata
    {
        uint16_t address;
        uint16_t size;
        uint16_t* ptr;
    };

    extern uint16_t registers[REGISTERS_SIZE];

    // EDIT HERE BEGIN

    // Declare pointers to structures of data we want to send/receive
    extern Vector3* cmd_vel;
    extern Vector3* measured_vel;
    extern StmConfig* stm_config;

    // Metadata to gather info needed to read/write the data (for master only)
    extern register_metadata reg_cmd_vel;
    extern register_metadata reg_measured_vel;
    extern register_metadata reg_stm_config;

    // EDIT HERE END

    uint16_t* init_ptr_to_register(uint16_t size);

    void init_register_metadata(register_metadata &reg_meta, uint16_t* ptr, uint16_t size);

    void setup_registers_slave();

    void setup_registers_master();

}



#endif //MODBUS_REGISTERS_H
