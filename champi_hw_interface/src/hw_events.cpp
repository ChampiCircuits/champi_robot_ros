#include "champi_hw_interface/hw_interface.h"


void HardwareInterfaceNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = msg;
}
