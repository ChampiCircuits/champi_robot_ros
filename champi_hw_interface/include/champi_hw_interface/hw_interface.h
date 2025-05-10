#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#define MODBUS_TIMEOUT_US 50000 // 50ms
#define MODBUS_MAX_RETRIES 5

#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
// #include <boost/asio.hpp> // Compiles without, but makes vscode happy

#include <string>
#include <vector>

#include <champi_hw_interface/ModbusRegister.h>
#include "tf2_ros/transform_broadcaster.h"

#include <champi_interfaces/msg/stm_state.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "util/ros_geometry.h"
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace com_types;

class PoseIntegrator {
public:
    PoseIntegrator() : pose_{0, 0, 0} {}

    void reset() {
        pose_ = Vector3{0, 0, 0};
    }

    Vector3 compute(const Vector3 &vel, double dt) {
        pose_.x += vel.x * cos(pose_.theta) * dt - vel.y * sin(pose_.theta) * dt;
        pose_.y += vel.x * sin(pose_.theta) * dt + vel.y * cos(pose_.theta) * dt;
        pose_.theta += vel.theta * dt;
        return pose_;
    }

private:
    Vector3 pose_;
};


class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode();
    ~HardwareInterfaceNode() override;

private:
    int setup_modbus();

    void write_config();
    void read_config();
    void setup_stm();

    void write( mod_reg::register_metadata &reg_meta) const;
    void read( mod_reg::register_metadata &reg_meta) const;
    void loop();

    void publish_transform();

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);
    void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void set_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void actuators_control_callback(const std_msgs::msg::Int8 msg) const;
    void check_for_actuators_state() const;
    void read_stm_state();

    nav_msgs::msg::Odometry make_odom_wheels(const Vector3 &vel, double dt);
    nav_msgs::msg::Odometry make_odom_otos(const Vector3 &pose, double dt) const;

    std::string device_ser_no_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    Config stm_config_{};

    geometry_msgs::msg::Twist latest_twist_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_otos_;
    std::vector<double> cov_pose_odom_otos_;
    std::vector<double> cov_vel_odom_otos_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_set_pose_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscriber_ctrl_actuators_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr pub_ctrl_actuators_;

    rclcpp::Publisher<champi_interfaces::msg::STMState>::SharedPtr pub_stm_state;
};


#endif // HW_INTERFACE_H