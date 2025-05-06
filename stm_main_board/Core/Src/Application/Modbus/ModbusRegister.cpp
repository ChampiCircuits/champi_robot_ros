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
Requests *requests;
Actuators *actuators;

#ifdef MODBUS_MASTER
// Metadata to gather info needed to read/write the data (for master only)
register_metadata reg_config;
register_metadata reg_state;
register_metadata reg_cmd;
register_metadata reg_requests;
register_metadata reg_actuators;

// EDIT HERE END

void init_register_metadata(register_metadata &reg_meta, uint16_t *ptr,
                            uint16_t n_bytes) { // size in bytes

  uint16_t n_coils = n_bytes / sizeof(uint16_t);
  if (n_bytes % sizeof(uint16_t) != 0) {
    n_coils += 1;
  }

  reg_meta.ptr = ptr;
  reg_meta.size = n_coils;
  reg_meta.address = ptr - registers;
}
#endif // MODBUS_MASTER

uint16_t *init_ptr_to_register(uint16_t n_bytes) { // size in bytes
  uint16_t n_coils = n_bytes / sizeof(uint16_t);
  if (n_bytes % sizeof(uint16_t) != 0) {
    n_coils += 1;
  }

  static uint16_t offset = 0;

  if (n_coils == 0) {
    printf("Error: Message length < 16 bits, please make the message bigger.\n");
    // TODO handle error
    return nullptr;
  }

  if (n_coils % 32 == 0) {
    printf("Error: Message length is multiple of 32, which is not handled "
           "properly for now. Please add dummy data to the message type.\n");
    // TODO handle error
    return nullptr;
  }

  if (offset + n_coils > REGISTERS_SIZE) {
    printf("Error: Not enough space in registers\n");
    // TODO handle error
    return nullptr;
  }
  uint16_t *ptr = registers + offset;
  offset += n_coils;
  return ptr;
}

void setup_registers() {

  // EDIT HERE BEGIN

  config = (Config *)init_ptr_to_register(sizeof(Config));
  state = (State *)init_ptr_to_register(sizeof(State));
  cmd = (Cmd *)init_ptr_to_register(sizeof(Cmd));
  requests = (Requests *)init_ptr_to_register(sizeof(Requests));
  actuators= (Actuators *)init_ptr_to_register(sizeof(Actuators));

#ifdef MODBUS_MASTER
  init_register_metadata(reg_config, (uint16_t *)config, sizeof(Config));
  init_register_metadata(reg_state, (uint16_t *)state, sizeof(State));
  init_register_metadata(reg_cmd, (uint16_t *)cmd,sizeof(Cmd));
  init_register_metadata(reg_requests, (uint16_t *)requests, sizeof(Requests));
  init_register_metadata(reg_actuators, (uint16_t *)actuators, sizeof(Actuators));

#endif // MODBUS_MASTER

  // EDIT HERE END
}

} // namespace mod_reg
