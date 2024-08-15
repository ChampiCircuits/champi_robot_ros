#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sensor_msgs/msg/laser_scan.hpp>

//CAN
#include "champi_can/can_ids.hpp"
#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"

using namespace std;

// LED RING
#define LED_COUNT 24


class FunNode : public rclcpp::Node
{
public:
    FunNode() : Node("fun_node"),
    champi_can_interface_(
                    "can0",
                    {can_ids::LED_RING_DISTANCES},
                    false) {
        // ================================ Get parameters =================================
        double loop_freq = this->declare_parameter<double>("loop_freq", 1.0);

        // ================================ Initialize ROS related ===================================

        // Subscriber for laser scan TODO QoS
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FunNode::laser_scan_callback, this, std::placeholders::_1)
        );

        // Timer for main loop
        loop_timer_ = this->create_wall_timer(
            1s/loop_freq, std::bind(&FunNode::loop_callback, this)
        );


        // Start the CAN interface
        int ret = champi_can_interface_.start();

        if(ret == 0) {
            RCLCPP_INFO(this->get_logger(), "CAN interface started successfully");

        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Error starting CAN interface");
            exit(1);// TODO handle error
        }

        
        RCLCPP_INFO(this->get_logger(), "Launched Fun Node !");
    }

private:
    // Subscriber for laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_msg;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr loop_timer_;

    // CAN
    ChampiCan champi_can_interface_;
    msgs_can::LedRingDistances led_ring_distances_msg;

    // ================================ Callbacks ===================================

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received laser scan");
        this->last_msg = msg;
    }

    void loop_callback() {
        if (this->last_msg == nullptr)
            return;
            
        float tab_distances[LED_COUNT];
        size_t s = this->last_msg->ranges.size();
        for (int i=0;i<LED_COUNT;i++) {
            tab_distances[i] = this->last_msg->ranges[i* s/24];
            std::cout << (int)(tab_distances[i]*100.)<<"\t";
        }
        std::cout << std::endl;
        send_by_CAN(tab_distances);
    }

    void send_by_CAN(float tab_distances[LED_COUNT]) {
        this->led_ring_distances_msg.clear_distances();
        for (int i=0; i<LED_COUNT;i++) {
            this->led_ring_distances_msg.add_distances(tab_distances[i]);
        }


        // Send message
        if(champi_can_interface_.send(can_ids::LED_RING_DISTANCES, led_ring_distances_msg.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FunNode>());
    rclcpp::shutdown();
    return 0;
}