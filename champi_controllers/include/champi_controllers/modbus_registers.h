
#ifndef MODBUS_REGISTERS_H

// Array containing the modbus data. Low level, we don't interact with it directly
#define REGISTERS_SIZE 100
uint16_t registers[REGISTERS_SIZE];

// Register abstraction, to simplify use of the data

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

struct register_metadata
{
    uint16_t address;
    uint16_t size;
    uint16_t* ptr;
};

void init_register_metadata(register_metadata &reg_meta, uint16_t* ptr, uint16_t size)
{
    reg_meta.ptr = ptr;
    reg_meta.size = size;
    reg_meta.address = ptr - registers;
}

// EDIT HERE BEGIN

// Define here the structures for the data we want to send/receive

struct Vector3
{
    float x;
    float y;
    float z;
};

// Declare pointers to our data structure.
Vector3* cmd_vel;
Vector3* measured_vel;

// Metadata to gather info needed to read/write the data (for master only)
register_metadata reg_cmd_vel;
register_metadata reg_measured_vel;

// EDIT HERE END


void setup_registers_slave(){

    // EDIT HERE BEGIN

    cmd_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));
    measured_vel = (Vector3*) init_ptr_to_register( sizeof(Vector3) / sizeof(uint16_t));

    // EDIT HERE END
}

void setup_registers_master(){

    setup_registers_slave();

    // EDIT HERE BEGIN

    init_register_metadata(reg_cmd_vel, (uint16_t*) cmd_vel, sizeof(Vector3) / sizeof(uint16_t));
    init_register_metadata(reg_measured_vel, (uint16_t*) measured_vel, sizeof(Vector3) / sizeof(uint16_t));

    // EDIT HERE END
}


#define MODBUS_REGISTERS_H

#endif //MODBUS_REGISTERS_H
