#include "champi_controllers/hw_interface.h"


int HardwareInterfaceNode::write( mod_reg::register_metadata &reg_meta)
{
    int rc = modbus_write_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data: error num: %d, message: %s", errno, modbus_strerror(errno));
    }
    return rc;
}

int HardwareInterfaceNode::read( mod_reg::register_metadata &reg_meta)
{
    int rc = modbus_read_registers(this->mb_, reg_meta.address, reg_meta.size, reg_meta.ptr);
    if (rc == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data: %s", modbus_strerror(errno));
    }
    return rc;

}

void HardwareInterfaceNode::loop()
{
     mod_reg::cmd_vel->x = 0.1;
     mod_reg::cmd_vel->y = 0.2;
     mod_reg::cmd_vel->theta = 0.3;


    write( mod_reg::reg_cmd_vel);

    read( mod_reg::reg_measured_vel);

    read( mod_reg::reg_otos_pose);

    RCLCPP_INFO(this->get_logger(), "Measured velocity: x=%f, y=%f, z=%f",  mod_reg::measured_vel->x,  mod_reg::measured_vel->y,  mod_reg::measured_vel->theta);
    RCLCPP_INFO(this->get_logger(), "Pose: x=%f, y=%f, z=%f",  mod_reg::otos_pose->x,  mod_reg::otos_pose->y,  mod_reg::otos_pose->theta);
}