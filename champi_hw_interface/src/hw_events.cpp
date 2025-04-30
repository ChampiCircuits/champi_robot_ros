#include "champi_hw_interface/hw_interface.h"

#include <tf2/LinearMath/Quaternion.hpp>

void HardwareInterfaceNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = *msg;
}

void HardwareInterfaceNode::initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_WARN(this->get_logger(), "Received set_pose request to %f, %f, %f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                tf2::getYaw(msg->pose.pose.orientation));

    initial_offset_ = msg->pose.pose;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, mod_reg::state->otos_pose.theta);
    offset_otos_when_set_pose_.position.x = mod_reg::state->otos_pose.x;
    offset_otos_when_set_pose_.position.y = mod_reg::state->otos_pose.y;
    offset_otos_when_set_pose_.orientation.x = q.x();
    offset_otos_when_set_pose_.orientation.y = q.y();
    offset_otos_when_set_pose_.orientation.z = q.z();
    offset_otos_when_set_pose_.orientation.w = q.w();

    pose_integrator_odom_wheels_.reset();

    mod_reg::requests->request_reset_otos = true; // actually this request calibrates only.
    this->write(mod_reg::reg_requests);
    RCLCPP_WARN(this->get_logger(), "Initial pose updated. Re-calibrating OTOS.");
}
