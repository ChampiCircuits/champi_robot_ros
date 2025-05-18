#include "champi_hw_interface/hw_interface.h"
#include "champi_hw_interface/hw_actuators.h"

#include <tf2/LinearMath/Quaternion.hpp>

void HardwareInterfaceNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = *msg;
}

void HardwareInterfaceNode::actuators_control_callback(const std_msgs::msg::Int8 msg) const
{
    int actuator_number = msg.data;
    RCLCPP_INFO(this->get_logger(), "New actuator command received! %d = %s", actuator_number, to_string(static_cast<ActuatorCommand>(actuator_number)).c_str());
    mod_reg::actuators->requests[actuator_number] = static_cast<uint8_t>(ActuatorState::REQUESTED);
    this->write(mod_reg::reg_actuators);
}