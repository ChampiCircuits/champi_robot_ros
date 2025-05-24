
#include <rclcpp/rclcpp.hpp>
#include "champi_hw_interface/hw_interface.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
