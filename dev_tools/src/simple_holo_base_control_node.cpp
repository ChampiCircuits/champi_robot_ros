#include <string>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"

#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"

using namespace std;

class SimpleHoloBaseControlNode : public rclcpp::Node
{
public:
    SimpleHoloBaseControlNode() : 
        Node("simple_holo_base_control_node"),
        champi_can_interface_(this->declare_parameter("can_interface_name", "vcan0"), {(int) this->declare_parameter("id_send", 0x10)})
    {

        // Get parameters
        string topic_twist_in = this->declare_parameter("topic_twist_in", "cmd_vel");
        string can_interface_name = this->get_parameter("can_interface_name").as_string();
        int id_send_cmd = this->get_parameter("id_send").as_int();
        int id_receive_vel = this->declare_parameter("id_receive", 0x11);

        // Print parameters
        RCLCPP_INFO(this->get_logger(), "Node started with the following parameters:");
        RCLCPP_INFO(this->get_logger(), "topic_twist_in: %s", topic_twist_in.c_str());
        RCLCPP_INFO(this->get_logger(), "id_send_cmd: %d", id_send_cmd);
        RCLCPP_INFO(this->get_logger(), "id_receive_vel: %d", id_receive_vel);

        champi_can_interface_.start();
        // Create Subscribers
        sub_twist_in_ = this->create_subscription<geometry_msgs::msg::Twist>(topic_twist_in, 10, std::bind(&SimpleHoloBaseControlNode::twist_in_callback, this, std::placeholders::_1));
        


    }

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_in_;

    // Variables
    ChampiCan champi_can_interface_;

    // Callbacks
    void twist_in_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        msgs_can::BaseVel base_vel_cmd;

        base_vel_cmd.set_x(msg->linear.x);
        base_vel_cmd.set_y(msg->linear.y);
        base_vel_cmd.set_theta(msg->angular.z);

        // Send message
        if(champi_can_interface_.send(0x10, base_vel_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
        }
    }



};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleHoloBaseControlNode>());
    rclcpp::shutdown();
    return 0;
}