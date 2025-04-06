#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <modbus/modbus.h>
#include <boost/asio.hpp> // Compiles without, but makes vscode happy

#include <string>
#include <vector>

#include <champi_controllers/ModbusRegister.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    void reconnect();

    void write_config();
    void read_config();
    void setup_stm();

    int write( mod_reg::register_metadata &reg_meta);
    int read( mod_reg::register_metadata &reg_meta);
    void loop();

    void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);

    nav_msgs::msg::Odometry make_odom_wheels(const Vector3 &vel, double dt);
    nav_msgs::msg::Odometry make_odom_otos(const Vector3 &pose, double dt);

    std::string device_ser_no_;
    int baud_rate_;
    int slave_id_;
    modbus_t *mb_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;

    StmConfig stm_config_{};

    geometry_msgs::msg::Twist::SharedPtr latest_twist_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_wheels_;
    PoseIntegrator pose_integrator_odom_wheels_;
    std::vector<double> cov_pose_odom_wheels_;
    std::vector<double> cov_vel_odom_wheels_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_otos_;
    std::vector<double> cov_pose_odom_otos_;
    std::vector<double> cov_vel_odom_otos_;

};


#endif // HW_INTERFACE_H