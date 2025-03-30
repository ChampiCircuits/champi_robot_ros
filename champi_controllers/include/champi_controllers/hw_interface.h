#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <boost/asio.hpp> // Compiles without, but makes vscode happy

#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <champi_controllers/ModbusRegister.h>

class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode();
    ~HardwareInterfaceNode();

private:
    
    void setup_modbus();

    void write_config();
    void read_config();
    void setup_stm();

    int write( mod_reg::register_metadata &reg_meta);
    int read( mod_reg::register_metadata &reg_meta);
    void loop();

    std::string device_ser_no_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    StmConfig stm_config_{};
};


#endif // HW_INTERFACE_H