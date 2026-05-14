#include "champi_hw_interface/hw_interface.h"
#include "champi_hw_interface/hw_actuators.h"

#include <tf2/LinearMath/Quaternion.hpp>

void HardwareInterfaceNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = *msg;
}

void HardwareInterfaceNode::actuators_control_callback(const std_msgs::msg::Int8 msg) const
{
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    int actuator_number = msg.data;
    RCLCPP_INFO(this->get_logger(), "New actuator command received! %d = %s", actuator_number, to_c_str(static_cast<ActuatorCommand>(actuator_number)));
    mod_reg::actuators->requests[actuator_number] = static_cast<uint8_t>(ActuatorState::REQUESTED);
    this->write(mod_reg::reg_actuators);
}

void HardwareInterfaceNode::nutboxes_detection_callback(const champi_interfaces::msg::NutBoxesDetection::SharedPtr msg)
{
    // Ignore "no detection" sentinel (z == -1.0)
    if (msg->pose.position.z == -1.0) {
        return;
    }
    // Ignore if not all 4 colors are known
    if (msg->colors.size() < 4) {
        // RCLCPP_WARN(this->get_logger(), "📦 Nutboxes detection: only %zu colors received, expected 4 — ignoring", msg->colors.size());
        return;
    }

    // NutBoxesDetection color constants: UNKNOWN=0, BLUE=1, YELLOW=2
    // TeamColor enum:                    UNKNOWN=0, YELLOW=1, BLUE=2
    // → COLOR_BLUE(1)   maps to TeamColor::BLUE(2)
    // → COLOR_YELLOW(2) maps to TeamColor::YELLOW(1)

    const TeamColor team_color = mod_reg::requests->team_color;

    // Build bitmask: bit i = 1 if nutbox[i] must be returned (different color from team)
    uint8_t mask = 0;
    for (int i = 0; i < 4; i++) {
        const uint8_t detected = msg->colors[i];
        if (detected == 0) continue; // COLOR_UNKNOWN → skip

        // Convert NutBoxesDetection color to TeamColor
        const TeamColor detected_team_color = (detected == 1) ? TeamColor::BLUE : TeamColor::YELLOW;

        if (detected_team_color != team_color) {
            mask |= static_cast<uint8_t>(1 << i); // bit i → return this nutbox
        }
    }

    // RCLCPP_INFO(this->get_logger(),
    //     "📦 Nutboxes detection: team_color=%d, mask=0b%d%d%d%d (cup3|cup2|cup1|cup0)",
    //     static_cast<int>(team_color),
    //     (mask >> 3) & 1, (mask >> 2) & 1, (mask >> 1) & 1, (mask >> 0) & 1);

    // Store mask as individual cup activation flags
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    mod_reg::actuators->left_suction_cup_0_activation = (mask >> 0) & 1;
    mod_reg::actuators->left_suction_cup_1_activation = (mask >> 1) & 1;
    mod_reg::actuators->left_suction_cup_2_activation = (mask >> 2) & 1;
    mod_reg::actuators->left_suction_cup_3_activation = (mask >> 3) & 1;
    mod_reg::actuators->right_suction_cup_0_activation = (mask >> 0) & 1;
    mod_reg::actuators->right_suction_cup_1_activation = (mask >> 1) & 1;
    mod_reg::actuators->right_suction_cup_2_activation = (mask >> 2) & 1;
    mod_reg::actuators->right_suction_cup_3_activation = (mask >> 3) & 1;

    this->write(mod_reg::reg_actuators);
}
