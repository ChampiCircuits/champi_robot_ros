#ifdef MODBUS_MASTER
#include "champi_hw_interface/ModbusRegister.h"
#else
#include "Application/Modbus/ModbusRegister.h"
#endif // MODBUS_MASTER
#include <cstdio>

namespace mod_reg {
uint16_t registers[REGISTERS_SIZE];

// EDIT HERE BEGIN

// Declare pointers to structures of data we want to send/receive
Config *config;
State *state;
Cmd *cmd;

#ifdef MODBUS_MASTER
// Metadata to gather info needed to read/write the data (for master only)
register_metadata reg_config;
register_metadata reg_state;
register_metadata reg_cmd;

// EDIT HERE END

void init_register_metadata(register_metadata &reg_meta, uint16_t *ptr,
                            uint16_t size) {
  reg_meta.ptr = ptr;
  reg_meta.size = size;
  reg_meta.address = ptr - registers;
}
#endif // MODBUS_MASTER

uint16_t *init_ptr_to_register(uint16_t size) {
  static uint16_t offset = 0;

  if (size % 64 == 0) {
    printf("Error: Message length is multiple of 64, which is not handled "
           "properly for now. Please add dummy data to the message type.\n");
    // TODO handle error
    return nullptr;
  }

  if (offset + size > REGISTERS_SIZE) {
    printf("Error: Not enough space in registers\n");
    // TODO handle error
    return nullptr;
  }
  uint16_t *ptr = registers + offset;
  offset += size;
  return ptr;
}

void setup_registers() {

  // EDIT HERE BEGIN

  config = (Config *)init_ptr_to_register(sizeof(Config) / sizeof(uint16_t));
  state = (State *)init_ptr_to_register(sizeof(State) / sizeof(uint16_t));
  cmd = (Cmd *)init_ptr_to_register(sizeof(Cmd) / sizeof(uint16_t));

#ifdef MODBUS_MASTER
  init_register_metadata(reg_config, (uint16_t *)config,
                         sizeof(Config) / sizeof(uint16_t));
  init_register_metadata(reg_state, (uint16_t *)state,
                         sizeof(State) / sizeof(uint16_t));
  init_register_metadata(reg_cmd, (uint16_t *)cmd,
                         sizeof(Cmd) / sizeof(uint16_t));

#endif // MODBUS_MASTER

  // EDIT HERE END
}

} // namespace mod_reg
