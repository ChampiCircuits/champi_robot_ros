#include "champi_controllers/hw_interface.h"


HardwareInterfaceNode::HardwareInterfaceNode() : Node("modbus_sender_node")
{
    this->declare_parameter<std::string>("device_ser_no", "3952366C3233");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("slave_id", 1);

    this->device_ser_no_ = this->get_parameter("device_ser_no").as_string();
    this->baud_rate_ = this->get_parameter("baud_rate").as_int();
    this->slave_id_ = this->get_parameter("slave_id").as_int();

    stm_config_.is_set = false;

    stm_config_.holo_drive_config.wheel_radius = (float) this->declare_parameter<float>("base_config.wheel_radius");
    stm_config_.holo_drive_config.base_radius = (float) this->declare_parameter<float>("base_config.base_radius");
    stm_config_.holo_drive_config.max_speed_linear = this->declare_parameter<double>("base_config.max_speed_linear");
    stm_config_.holo_drive_config.max_accel_wheel = (float) this->declare_parameter<float>("base_config.max_accel_wheel");
    stm_config_.holo_drive_config.max_accel_linear = this->declare_parameter<double>("base_config.max_acceleration_linear");
    stm_config_.holo_drive_config.max_decel_linear = this->declare_parameter<double>("base_config.max_deceleration_linear");
    stm_config_.holo_drive_config.max_accel_angular = this->declare_parameter<double>("base_config.max_acceleration_angular");
    stm_config_.holo_drive_config.max_decel_angular = this->declare_parameter<double>("base_config.max_deceleration_angular");

    stm_config_.cmd_vel_timeout = (float) this->declare_parameter<float>("base_config.cmd_vel_timeout");

    mod_reg::setup_registers();

    setup_modbus();

    setup_stm();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&HardwareInterfaceNode::loop, this));

}

HardwareInterfaceNode::~HardwareInterfaceNode()
{
    if (mb_) {
        modbus_close(mb_);
        modbus_free(mb_);
    }
}
