#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#define MODBUS_TIMEOUT_US 50000 // 50ms
#define MODBUS_MAX_RETRIES 5

#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <boost/asio.hpp> // Compiles without, but makes vscode happy

#include <string>
#include <vector>

#include <champi_hw_interface/ModbusRegister.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);
    void initial_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

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

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_wheels_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_wheels_;
    PoseIntegrator pose_integrator_odom_wheels_;
    std::vector<double> cov_pose_odom_wheels_;
    std::vector<double> cov_vel_odom_wheels_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_otos_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_otos_;
    std::vector<double> cov_pose_odom_otos_;
    std::vector<double> cov_vel_odom_otos_;

    geometry_msgs::msg::Pose offset_otos_; // For handling initial pose without actually resetting the OTOS pose.
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_initial_pose_;

};


#endif // HW_INTERFACE_H