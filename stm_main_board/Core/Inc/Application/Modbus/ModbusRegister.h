#ifndef MODBUS_REGISTERS_H
#define MODBUS_REGISTERS_H

// #include <sys/_stdint.h>
#ifdef MODBUS_MASTER
#include <champi_hw_interface/DataStructures.h>
#else
#include <Application/Modbus/DataStructures.h>
#endif // MODBUS_MASTER
#include <stdint.h>

#define REGISTERS_SIZE 500

namespace mod_reg {

struct register_metadata {
  uint16_t address;
  uint16_t size;
  uint16_t *ptr;
};

extern uint16_t registers[REGISTERS_SIZE];

using namespace com_types;

// EDIT HERE BEGIN

// Declare pointers to structures of data we want to send/receive

extern Config *config;
extern State *state;
extern Cmd *cmd;
extern Requests *requests;

#ifdef MODBUS_MASTER
// Metadata to gather info needed to read/write the data (for master only)
extern register_metadata reg_config;
extern register_metadata reg_state;
extern register_metadata reg_cmd;
extern register_metadata reg_requests;

// EDIT HERE END

void init_register_metadata(register_metadata &reg_meta, uint16_t *ptr,
                            uint16_t size);
#endif // MODBUS_MASTER

uint16_t *init_ptr_to_register(uint16_t size);

void setup_registers();

} // namespace mod_reg

#endif // MODBUS_REGISTERS_H
