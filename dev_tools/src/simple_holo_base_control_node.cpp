#include <string>

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <champi_can/can_ids.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"

using namespace std;

class SimpleHoloBaseControlNode : public rclcpp::Node
{
public:
    SimpleHoloBaseControlNode() : 
        Node("simple_holo_base_control_node"),
        champi_can_interface_(
                this->declare_parameter("can_interface_name", "vcan0"),
                {can_ids::BASE_CURRENT_VEL, can_ids::BASE_RET_CONFIG},
                this->declare_parameter("champi_can_verbose_mode", true))
    {

        // Get parameters
        string topic_twist_in = this->declare_parameter("topic_twist_in", "cmd_vel");
        string can_interface_name = this->get_parameter("can_interface_name").as_string();

        base_config_.max_accel = this->declare_parameter("max_accel", 10.0);
        base_config_.wheel_radius = this->declare_parameter("wheel_radius", 0.029);
        base_config_.base_radius = this->declare_parameter("base_radius", 0.175);
        base_config_.cmd_vel_timeout = this->declare_parameter("cmd_vel_timeout", 0.1);


        // Print parameters
        RCLCPP_INFO(this->get_logger(), "Node started with the following parameters:");
        RCLCPP_INFO(this->get_logger(), "topic_twist_in: %s", topic_twist_in.c_str());


        // Initialize current_pose_
        current_pose_.x = 0.0;

        // Start the CAN interface
        int ret = champi_can_interface_.start();

        if(ret == 0) {
            RCLCPP_INFO(this->get_logger(), "CAN interface started successfully");

        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Error starting CAN interface");
            // TODO handle error
        }

        // Send config to the base
        waiting_for_ret_config = true;
        send_config();

        // Create Subscribers
        sub_twist_in_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(topic_twist_in, 10, std::bind(&SimpleHoloBaseControlNode::twist_in_callback, this, std::placeholders::_1));
    
        // Create Publishers
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

            // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&SimpleHoloBaseControlNode::loop_callback, this));

    }

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_in_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    // TF broadcast related
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    ChampiCan champi_can_interface_;
    msgs_can::BaseVel current_vel_;
    struct Pose {
        float x;
        float y;
        float theta;
    } current_pose_;
    bool waiting_for_ret_config;

    // Parameters
    struct BaseConfig {
        float max_accel;
        float wheel_radius;
        float base_radius;
        float cmd_vel_timeout;
    } base_config_;

    // Functions

    void send_config() {
        msgs_can::BaseConfig base_set_config;
        base_set_config.set_max_accel(base_config_.max_accel);
        base_set_config.set_wheel_radius(base_config_.wheel_radius);
        base_set_config.set_base_radius(base_config_.base_radius);

        // Send message
        if(champi_can_interface_.send(can_ids::BASE_SET_CONFIG, base_set_config.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending config message");
            // TODO send diagnostic
        }
    }

    void rx_can() {
        if(champi_can_interface_.check_if_new_full_msg(can_ids::BASE_CURRENT_VEL)) {

            auto buffer = champi_can_interface_.get_full_msg(can_ids::BASE_CURRENT_VEL);

            // Update current_vel_
            current_vel_.ParseFromString(buffer);
        }
        if(waiting_for_ret_config && champi_can_interface_.check_if_new_full_msg(can_ids::BASE_RET_CONFIG)) {
            waiting_for_ret_config = false;

            auto buffer = champi_can_interface_.get_full_msg(can_ids::BASE_RET_CONFIG);
            msgs_can::BaseConfig base_ret_config;
            base_ret_config.ParseFromString(buffer);

            RCLCPP_INFO(this->get_logger(), "Base confirmed config: max_accel: %f, wheel_radius: %f, base_radius: %f",
                        base_ret_config.max_accel(), base_ret_config.wheel_radius(), base_ret_config.base_radius());

        }
    }



    // Callbacks
    void twist_in_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        msgs_can::BaseVel base_vel_cmd;

        base_vel_cmd.set_x(msg->twist.linear.x);
        base_vel_cmd.set_y(msg->twist.linear.y);
        base_vel_cmd.set_theta(msg->twist.angular.z);

        // Send message
        if(champi_can_interface_.send(can_ids::BASE_CMD_VEL, base_vel_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }

    }


    void loop_callback() {
        // Read incoming message if available
        rx_can();

        if(waiting_for_ret_config) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for base to confirm config");
            return;
        }

        // Update pose / velocity for other nodes
        update_pose();
        publish_odom();
        broadcast_tf();

    }

    void update_pose() {
        // Update current_pose_ by integrating current_vel_

        static bool first_time = true;
        static rclcpp::Time last_time;
        if(first_time) {
            last_time = this->now();
            first_time = false;
            return;
        }

        rclcpp::Duration dt = this->now() - last_time;
        last_time = this->now();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "dt: %f", dt.seconds());

        // Robot speed is expressed relative to the robot frame, so we need to transform it to the world frame

        current_pose_.x += current_vel_.x() * cos(current_pose_.theta) * dt.seconds() - current_vel_.y() * sin(current_pose_.theta) * dt.seconds();
        current_pose_.y += current_vel_.x() * sin(current_pose_.theta) * dt.seconds() + current_vel_.y() * cos(current_pose_.theta) * dt.seconds();
        current_pose_.theta += current_vel_.theta() * dt.seconds();
    }

    void publish_odom() {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = current_pose_.x;
        msg.pose.pose.position.y = current_pose_.y;
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_pose_.theta);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        msg.twist.twist.linear.x = this->current_vel_.x();
        msg.twist.twist.linear.y = this->current_vel_.y();
        msg.twist.twist.angular.z = this->current_vel_.theta();

        pub_odom_->publish(msg);
    }

    void broadcast_tf() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = current_pose_.x;
        transformStamped.transform.translation.y = current_pose_.y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_pose_.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
    
        tf_broadcaster_->sendTransform(transformStamped);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleHoloBaseControlNode>());
    rclcpp::shutdown();
    return 0;
}