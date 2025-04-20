#include "champi_hw_interface/hw_interface.h"

void HardwareInterfaceNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = *msg;
}

void HardwareInterfaceNode::initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    initial_pose_ = *msg;

    pose_integrator_odom_wheels_.reset();

    mod_reg::requests->request_reset_otos = true;
    this->write(mod_reg::reg_requests);
    RCLCPP_INFO(this->get_logger(), "Initial pose updated. Resetting OTOS");
}
