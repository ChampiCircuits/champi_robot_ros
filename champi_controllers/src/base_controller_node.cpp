#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"
#include "champi_can/can_ids.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#define CONFIG_RETRY_DELAY 1.0 // If no ret config reveived for this amount of time (s), we retry to send config

using namespace std;

class BaseControllerNode : public rclcpp::Node
{
public:
    BaseControllerNode() :
            Node("simple_holo_base_control_node"),
            diag_updater_base_(this),
            diag_updater_node_(this),
            champi_can_interface_(
                    this->declare_parameter<std::string>("can_interface_name"),
                    {can_ids::BASE_CURRENT_VEL, can_ids::BASE_RET_CONFIG, can_ids::BASE_STATUS},
                    this->declare_parameter<bool>("champi_can_verbose_mode"))
    {

        // Get parameters
        string topic_twist_in = this->declare_parameter("topic_twist_in", "cmd_vel");
        string can_interface_name = this->get_parameter("can_interface_name").as_string();
        timeout_connexion_stm_ = this->declare_parameter<double>("timeout_connexion_stm");
        timeout_connexion_ros_ = this->declare_parameter<double>("timeout_connexion_ros");

        base_config_.max_accel = (float) this->declare_parameter<float>("base_config.max_accel_wheels");
        base_config_.wheel_radius = (float) this->declare_parameter<float>("base_config.wheel_radius");
        base_config_.base_radius = (float) this->declare_parameter<float>("base_config.base_radius");
        base_config_.cmd_vel_timeout = (float) this->declare_parameter<float>("base_config.cmd_vel_timeout");

        // Print parameters
        // RCLCPP_INFO(this->get_logger(), "Node started with the following parameters:");
        // RCLCPP_INFO(this->get_logger(), "topic_twist_in: %s", topic_twist_in.c_str());
        // RCLCPP_INFO(this->get_logger(), "can_interface_name: %s", can_interface_name.c_str());
        // RCLCPP_INFO(this->get_logger(), "base_config.max_accel_wheels: %f", base_config_.max_accel);
        // RCLCPP_INFO(this->get_logger(), "base_config.wheel_radius: %f", base_config_.wheel_radius);
        // RCLCPP_INFO(this->get_logger(), "base_config.base_radius: %f", base_config_.base_radius);
        // RCLCPP_INFO(this->get_logger(), "base_config.cmd_vel_timeout: %f", base_config_.cmd_vel_timeout);

        // Set up diagnostics
        diag_updater_base_.setHardwareID("holo_base");
        diag_updater_base_.add("base_state", this, &BaseControllerNode::test_base_status);

        diag_updater_node_.setHardwareID("none");
        diag_updater_node_.add("node_state", this, &BaseControllerNode::test_node_state);

        // Initialize current_pose_
        current_pose_.x = 0.0;

        // Start the CAN interface
        int ret = champi_can_interface_.start();

        if(ret == 0) {
            RCLCPP_INFO(this->get_logger(), "CAN interface started successfully");

        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Error starting CAN interface");
            exit(1);// TODO handle error
        }

        // Reset the base
        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Resetting the base");
        reset_base();

        // Wait a little to let the base reset
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        send_config();
        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for base to confirm config");

        // Create Subscribers
        sub_twist_in_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(topic_twist_in, 10, std::bind(&BaseControllerNode::twist_in_callback, this, std::placeholders::_1));

        // Create Publishers
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&BaseControllerNode::loop_callback, this));

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

    // Diagnostic
    diagnostic_updater::Updater diag_updater_base_;
    diagnostic_updater::Updater diag_updater_node_;

    rclcpp::Time last_rx_status_time_;
    rclcpp::Time last_rx_twist_time_;
    msgs_can::Status current_status_;
    double dt_measured_{};

    bool timeout_connexion_stm_exceeded_ = false;
    bool timeout_connexion_ros_exceeded_ = false;
    bool got_first_twist_ = false;
    bool got_first_status_ = false;
    bool correct_config_received_ = false;
    // Structure to store the state of the node. Matches the ROS2 diagnostic msg to be able to publish it easily.
    struct NodeState {
        unsigned char lvl;
        std::string summary;
    } node_state_{};

    // Other variables
    ChampiCan champi_can_interface_;
    msgs_can::BaseVel current_vel_;
    geometry_msgs::msg::TwistStamped::SharedPtr latest_twist_;
    struct Pose {
        double x;
        double y;
        double theta;
    } current_pose_{};

    // Parameters
    struct BaseConfig {
        float max_accel;
        float wheel_radius;
        float base_radius;
        float cmd_vel_timeout;
    } base_config_{};
    double timeout_connexion_stm_; // receive status from CAN
    double timeout_connexion_ros_; // receive twist from ROS





    // =========================== Diagnostic tests (observation only, for /diagnostics) ===============================

    void test_base_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        // Summary
        if(!got_first_status_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No status received yet");
        }
        else if(timeout_connexion_stm_exceeded_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Timeout connexion to base exceeded");
        }
        else if(current_status_.has_status()) {
            switch(current_status_.status()) {
                case msgs_can::Status_StatusType::Status_StatusType_OK:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
                    break;
                case msgs_can::Status_StatusType::Status_StatusType_WARN:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning");
                    break;
                case msgs_can::Status_StatusType::Status_StatusType_ERROR:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error");
                    break;
                case msgs_can::Status_StatusType::Status_StatusType_INIT:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Initializing");
                    break;
                default:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown status");
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Maybe an error in the code, we should not enter this else.");
        }


        // Values
        if(current_status_.has_status()) {
            stat.add("base_status", msgs_can::Status_StatusType_Name(current_status_.status()));
        }
        if(current_status_.has_error()) {
            stat.add("base_error", msgs_can::Status_ErrorType_Name(current_status_.error()));
        }
    }

    void test_node_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        stat.summary(node_state_.lvl, node_state_.summary);
        stat.add("current_loop_rate (Hz)", 1.0 / dt_measured_);
        stat.add("got_first_status", got_first_status_);
        stat.add("got_first_twist", got_first_twist_);
        stat.add("correct_config_received", correct_config_received_);
        stat.add("timeout_connexion_stm_exceeded", timeout_connexion_stm_exceeded_);
        stat.add("timeout_connexion_ros_exceeded", timeout_connexion_ros_exceeded_);
    }






    // ======================== Real diagnostics, that can change the state of the node ================================

    void update_node_state(unsigned char lvl, std::string summary) {
        node_state_.lvl = lvl;
        node_state_.summary = summary;
        // Print with the correct color
        switch(lvl) {
            case diagnostic_msgs::msg::DiagnosticStatus::OK:
                RCLCPP_INFO(this->get_logger(), summary.c_str());
                break;
            case diagnostic_msgs::msg::DiagnosticStatus::WARN:
                RCLCPP_WARN(this->get_logger(), summary.c_str());
                break;
            case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
                RCLCPP_ERROR(this->get_logger(), summary.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown diagnostic level");
        }
    }

    /**
     *
     * @return true  if we can continue executing the loop, false if we should return
     */
    bool check_node_ok_and_update() {

        if(node_state_.lvl != diagnostic_msgs::msg::DiagnosticStatus::OK && 
            node_state_.lvl != diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            // Node is not OK. Do not change it.
            return false;
        }

        if(!got_first_status_ || !got_first_twist_ || !correct_config_received_) {
            // That's fine but we signal we should not execute the loop
            return false;
        }

        auto now = this->now();

        timeout_connexion_stm_exceeded_ = now - last_rx_status_time_ > rclcpp::Duration::from_seconds(timeout_connexion_stm_);
        timeout_connexion_ros_exceeded_ = now - last_rx_twist_time_ > rclcpp::Duration::from_seconds(timeout_connexion_ros_);

        if(timeout_connexion_stm_exceeded_ || timeout_connexion_ros_exceeded_) {
            // Set node state to ERROR
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Timeout error");
            diag_updater_node_.force_update(); // Send diagnostic right away
            return false;
        }

        // check if base status ok
        if(current_status_.has_status() && current_status_.status() != msgs_can::Status_StatusType::Status_StatusType_OK) {
            // Base not OK. Set node state to ERROR
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Base status not OK");
            diag_updater_node_.force_update();
            return false;
        }

        // Everything is OK

        if(node_state_.lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN) { // WARN is during initialization TODO pas bien si on veut utilier WARN pour autre chose aussi
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
            diag_updater_node_.force_update();
        }
        return true;
    }





    // ================================ Communication with the base (CAN bus) ==========================================

    void reset_base() {
        // Send message
        if(champi_can_interface_.send_raw(can_ids::BASE_RESET, "") != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending reset message");
            // TODO send diagnostic
            exit(1);
        }
    }

    void send_config() {
        msgs_can::BaseConfig base_set_config;
        base_set_config.set_max_accel(base_config_.max_accel);
        base_set_config.set_wheel_radius(base_config_.wheel_radius);
        base_set_config.set_base_radius(base_config_.base_radius);
        base_set_config.set_cmd_vel_timeout(base_config_.cmd_vel_timeout);

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
        if(!correct_config_received_ && champi_can_interface_.check_if_new_full_msg(can_ids::BASE_RET_CONFIG)) {

            auto buffer = champi_can_interface_.get_full_msg(can_ids::BASE_RET_CONFIG);
            msgs_can::BaseConfig base_ret_config;
            base_ret_config.ParseFromString(buffer);

            // Check if the config has all it's fields and they are the same as the sent ones
            if(!base_ret_config.has_max_accel() ||
                    !base_ret_config.has_wheel_radius() ||
                    !base_ret_config.has_base_radius() ||
                    !base_ret_config.has_cmd_vel_timeout()) {

                // Update / send diagnostic
                update_node_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Base returned config message is missing fields");
                diag_updater_node_.force_update();
            }
            // Received config is different from the sent one
            else if(base_ret_config.max_accel() != base_config_.max_accel ||
                    base_ret_config.wheel_radius() != base_config_.wheel_radius ||
                    base_ret_config.base_radius() != base_config_.base_radius ||
                    base_ret_config.cmd_vel_timeout() != base_config_.cmd_vel_timeout) {

                update_node_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Base returned config message is different from the sent one");
                diag_updater_node_.force_update();
            }
            // Config confirmed, everything is OK
            else {
                correct_config_received_ = true;
                update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Base confirmed config. Node still Initializing");
            }
        }
        if(champi_can_interface_.check_if_new_full_msg(can_ids::BASE_STATUS)) {
            last_rx_status_time_ = this->now();
            if(!got_first_status_) {
                got_first_status_ = true;
            }
            auto buffer = champi_can_interface_.get_full_msg(can_ids::BASE_STATUS);
            current_status_.ParseFromString(buffer); // Update current_status_
        }
    }

    void send_latest_twist_can() {
        msgs_can::BaseVel base_vel_cmd;
        base_vel_cmd.set_x((float) latest_twist_->twist.linear.x);
        base_vel_cmd.set_y((float) latest_twist_->twist.linear.y);
        base_vel_cmd.set_theta((float) latest_twist_->twist.angular.z);

        // Send message
        if(champi_can_interface_.send(can_ids::BASE_CMD_VEL, base_vel_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }
    }




    //  =========================================== Communication ROS2 =================================================

    void twist_in_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_rx_twist_time_ = this->now();
        if(!got_first_twist_) {
            got_first_twist_ = true;
        }
        latest_twist_ = msg;
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







    // =========================================== Main loop ===========================================================

    void loop_callback() {

        auto now = this->now();

        // Return if first time (to initialize last_time)
        static bool first_time = true;
        static rclcpp::Time last_time;
        if(first_time) {
            last_time = now;
            first_time = false;
            return;
        }

        // Update dt
        dt_measured_ = (now - last_time).seconds();
        last_time = now;

        static auto time_config_sent = now;
        if(!correct_config_received_ && now - time_config_sent > rclcpp::Duration::from_seconds(CONFIG_RETRY_DELAY)) {
            // Re-send config to the base
            send_config();
            RCLCPP_WARN(this->get_logger(), "Re-sending config to the base");
            time_config_sent = now;
        }


        // Read incoming message if available
        rx_can();

        // Return if needed (state not ok or 1st twist not received or 1st status not received)
        if(!check_node_ok_and_update()) {
            return;
        }

        // Send latest twist to the base
        send_latest_twist_can();

        // Update pose / velocity for other nodes
        update_pose();
        publish_odom();
        broadcast_tf();
    }


    // ===================================== Math / Helper functions ===================================================

    void update_pose() {
        // Update current_pose_ by integrating current_vel_

        // Robot speed is expressed relative to the robot frame, so we need to transform it to the world frame

        current_pose_.x += current_vel_.x() * cos(current_pose_.theta) * dt_measured_ - current_vel_.y() * sin(current_pose_.theta) * dt_measured_;
        current_pose_.y += current_vel_.x() * sin(current_pose_.theta) * dt_measured_ + current_vel_.y() * cos(current_pose_.theta) * dt_measured_;
        current_pose_.theta += current_vel_.theta() * dt_measured_;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseControllerNode>());
    rclcpp::shutdown();
    return 0;
}