#include <Modbus/ModbusRegister.h>
#include "usart.h"

#include "Application/ModbusTask.h"



modbusHandler_t ModbusH;


void ModbusTaskStart()
{
    mod_reg::setup_registers_slave();

    ModbusH.uModbusType = MB_SLAVE;
    ModbusH.port =  NULL; // &huart1 for UART
    ModbusH.u8id = 1; //Modbus slave ID
    ModbusH.u16timeOut = 1000;
    ModbusH.EN_Port = NULL;
    ModbusH.u16regs = mod_reg::registers;
    ModbusH.u16regsize= REGISTERS_SIZE;
    ModbusH.xTypeHW = USB_CDC_HW; // USART_HW for UART

    //Initialize Modbus library, and starts the modbus task
    ModbusInit(&ModbusH);
    // This function is weirdly named. It actualy performs configuration checks and does while(1) if bad configuration.
    // It doesn't start anything.
    ModbusStartCDC(&ModbusH);

}


