#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

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
                    {can_ids::BASE_CURRENT_VEL, can_ids::BASE_RET_CONFIG, can_ids::BASE_STATUS, can_ids::TRACKING_SENSOR_DATA, can_ids::TRACKING_SENSOR_STD},
                    this->declare_parameter<bool>("champi_can_verbose_mode"))
    {

        // Get parameters
        string topic_twist_in = this->declare_parameter("topic_twist_in", "cmd_vel");
        string can_interface_name = this->get_parameter("can_interface_name").as_string();
        timeout_connexion_stm_ = this->declare_parameter<double>("timeout_connexion_stm");
        timeout_connexion_ros_ = this->declare_parameter<double>("timeout_connexion_ros");

        double start_x = this->declare_parameter<double>("start_x");
        double start_y = this->declare_parameter<double>("start_y");
        double start_theta = this->declare_parameter<double>("start_theta");

        base_config_.max_accel = (float) this->declare_parameter<float>("base_config.max_accel_wheels");
        base_config_.wheel_radius = (float) this->declare_parameter<float>("base_config.wheel_radius");
        base_config_.base_radius = (float) this->declare_parameter<float>("base_config.base_radius");
        base_config_.cmd_vel_timeout = (float) this->declare_parameter<float>("base_config.cmd_vel_timeout");

        enable_accel_limit_ = this->declare_parameter<bool>("enable_accel_limits");
        max_accel_linear_ = this->declare_parameter<double>("max_acceleration_linear");
        max_decel_linear_ = this->declare_parameter<double>("max_deceleration_linear");
        max_accel_angular_ = this->declare_parameter<double>("max_acceleration_angular");
        max_decel_angular_ = this->declare_parameter<double>("max_deceleration_angular");
        max_speed_ = this->declare_parameter<double>("max_linear_speed");

        // Get covariances
        cov_pose_ = this->declare_parameter<std::vector<double>>("covariances.pose");
        // Check if the covariance is the right size (6 values = the diagonal of the covariance matrix)
        if (cov_pose_.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "covariances.pose must have 6 values (diagonal of the covariance matrix). Exiting.");
            return;
        }
        cov_vel_ = this->declare_parameter<std::vector<double>>("covariances.velocity");
        // Check if the covariance is the right size (6 values = the diagonal of the covariance matrix)
        if (cov_vel_.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "covariances.velocity must have 6 values (diagonal of the covariance matrix). Exiting.");
            return;
        }

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
        diag_updater_base_.add("base_state", this, &BaseControllerNode::test_stm_status);

        diag_updater_node_.setHardwareID("none");
        diag_updater_node_.add("node_state", this, &BaseControllerNode::test_node_state);

        // Initialize current_pose_
        current_pose_.x = start_x;
        current_pose_.y = start_y;
        current_pose_.theta = start_theta;

        //17.5

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
        sub_twist_in_ = this->create_subscription<geometry_msgs::msg::Twist>(topic_twist_in, 10, std::bind(
                &BaseControllerNode::twist_in_callback, this, std::placeholders::_1));
        
        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(
                &BaseControllerNode::initial_pose_callback, this, std::placeholders::_1));

        sub_do_reset_and_calibrate_ = this->create_subscription<std_msgs::msg::Empty>("/resetAndCalibrateTracking", 10, std::bind(
                &BaseControllerNode::send_reset_command_tracking, this, std::placeholders::_1));

        // Create Publishers
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_twist_limited_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_limited", 10);
        pub_odom_tracking_sensor_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("odom_tracking", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&BaseControllerNode::loop_callback, this));

    }

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_in_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_do_reset_and_calibrate_;


    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_odom_tracking_sensor_;
    // Publisher for cmd vel after limits have been applied
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_limited_;

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
    geometry_msgs::msg::Twist::SharedPtr latest_twist_;
    struct Pose {
        double x;
        double y;
        double theta;
    } current_pose_{};
    msgs_can::TrackingSensorData current_tracking_pose_;
    msgs_can::TrackingSensorStd current_tracking_std_;

    // Parameters
    struct BaseConfig {
        float max_accel;
        float wheel_radius;
        float base_radius;
        float cmd_vel_timeout;
    } base_config_{};
    double timeout_connexion_stm_; // receive status from CAN
    double timeout_connexion_ros_; // receive twist from ROS
    std::vector<double> cov_pose_;
    std::vector<double> cov_vel_;
    double max_speed_;

    bool enable_accel_limit_;
    double max_accel_linear_;
    double max_decel_linear_;
    double max_accel_angular_;
    double max_decel_angular_;






    // =========================== Diagnostic tests (observation only, for /diagnostics) ===============================

    void test_stm_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
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

        // If node state is not OK or WARN, leave it as is and return.
        if(node_state_.lvl != diagnostic_msgs::msg::DiagnosticStatus::OK && 
            node_state_.lvl != diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            // Node is not OK. Do not change it.
            return false;
        }

        // If the STM is not initialized yet, we should not execute the loop so we return false.
        if(!got_first_status_ || !correct_config_received_) {
            return false;
        }

        auto now = this->now();

        // Check if timeout for connexion to the base is exceeded. If yes, set node state to ERROR and return false.
        timeout_connexion_stm_exceeded_ = now - last_rx_status_time_ > rclcpp::Duration::from_seconds(timeout_connexion_stm_);
        if(timeout_connexion_stm_exceeded_) {
            // Don't do anything to the node state but report issue
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Timeout error (CAN bus). Elapsed time since last status: " + to_string((now - last_rx_status_time_).seconds()));
            diag_updater_node_.force_update(); // Send diagnostic right away
            //return true;
        }

        // Check if timeout for connexion to ROS is exceeded. If yes, set node state to ERROR and return false.
        // If we have not received the first twist yet, we do not check this timeout.
        if(got_first_twist_) {
            timeout_connexion_ros_exceeded_ = now - last_rx_twist_time_ > rclcpp::Duration::from_seconds(timeout_connexion_ros_);
            if(timeout_connexion_ros_exceeded_) {
                // Don't do anything to the node state but report issue
                update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Timeout error (ROS). Elapsed time since last twist: " + to_string((now - last_rx_twist_time_).seconds()));
                diag_updater_node_.force_update(); // Send diagnostic right away
                //return true;
            }
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
        return got_first_twist_; // If we have not received the first twist yet, we do not send commands to the base.
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

            // TODO quick fix -y and -theta
            current_vel_.set_y(-current_vel_.y());
            current_vel_.set_theta(-current_vel_.theta());
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
        if (champi_can_interface_.check_if_new_full_msg(can_ids::TRACKING_SENSOR_DATA)) {
            auto buffer = champi_can_interface_.get_full_msg(can_ids::TRACKING_SENSOR_DATA);
            current_tracking_pose_.ParseFromString(buffer); // Update tracking sensor position
            // RCLCPP_INFO(this->get_logger(), "new odom tracking data");
        }
        if (champi_can_interface_.check_if_new_full_msg(can_ids::TRACKING_SENSOR_STD)) {
            auto buffer = champi_can_interface_.get_full_msg(can_ids::TRACKING_SENSOR_STD);
            current_tracking_std_.ParseFromString(buffer); // Update tracking sensor std
            // RCLCPP_INFO(this->get_logger(), "new odom tracking std");
        }  
    }

    void send_twist_can(const geometry_msgs::msg::Twist::SharedPtr& twist) {
        msgs_can::BaseVel base_vel_cmd;
                // TODO quick fix -y and -theta
        base_vel_cmd.set_x((float) twist->linear.x);
        base_vel_cmd.set_y((float) -twist->linear.y);
        base_vel_cmd.set_theta((float) -twist->angular.z);

        // Send message
        if(champi_can_interface_.send(can_ids::BASE_CMD_VEL, base_vel_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }
    }

    void send_reset_command_tracking(const std_msgs::msg::Empty) {
        msgs_can::ResetAndCalibrateTrackingSensor reset_tracking_cmd;
        reset_tracking_cmd.set_reset(true);
        reset_tracking_cmd.set_calibrate(true);

        // Send message
        if(champi_can_interface_.send(can_ids::RESET_AND_CALIBRATE_TRACKING_SENSOR, reset_tracking_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }
    }


    //  =========================================== Communication ROS2 =================================================

    void twist_in_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_rx_twist_time_ = this->now();
        if(!got_first_twist_) {
            got_first_twist_ = true;
        }

        latest_twist_ = msg;
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg) {
        current_pose_.x = msg.pose.pose.position.x;
        current_pose_.y = msg.pose.pose.position.y;
        
        // get yaw from quaternion
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

        current_pose_.theta = tf2::impl::getYaw(q);
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
 

        // Covariance
        for(int i = 0; i < 6; i++) {
            msg.pose.covariance[i * 6 + i] = cov_pose_[i];
            msg.twist.covariance[i * 6 + i] = cov_vel_[i];
        }

        pub_odom_->publish(msg);
    }

    void publish_odom_tracking() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x = current_tracking_pose_.pose_x_mm();
        msg.pose.pose.position.y = current_tracking_pose_.pose_y_mm();
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_tracking_pose_.theta_rad());
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        // Covariance
        msg.pose.covariance[0] = current_tracking_std_.pose_x_std();
        msg.pose.covariance[6+1] = current_tracking_std_.pose_y_std();
        msg.pose.covariance[6*6-1] = current_tracking_std_.theta_std();

        pub_odom_tracking_sensor_->publish(msg);

        RCLCPP_WARN(this->get_logger(), "angle= %f", 2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)/M_PI*180.);
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

    void broadcast_tf_map() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
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

        // Check if we can send commands to the base. We can't if:
        // - Initialization is not finished
        // - we have not received the first twist yet (which is not part of the initialization)
        // - the node is OK or WARN
        bool allow_control_robot = check_node_ok_and_update();


        // If node is in error state, reset the node
        if(node_state_.lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {

            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "An error occured. The node will reset and and try to restart the base.");

            // Set node state to initializing
            correct_config_received_ = false;
            got_first_twist_ = false;

            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Resetting the base");
            reset_base();

            // Wait a little to let the base reset
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            send_config();
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for base to confirm config");
        }

        // If we can send commands to the base, do it
        if(allow_control_robot) {
            // Compute the limited speed
            auto limited_twist = std::make_shared<geometry_msgs::msg::Twist>();
            compute_limited_speed(limited_twist);
            // Send latest twist to the base
            send_twist_can(limited_twist);
            // Publish the limited twist
            pub_twist_limited_->publish(*limited_twist);
            // Update pose / velocity for other nodes
            update_pose();
        }

        // We always publish the odometry and broadcast the tf for the other nodes to initialize
        publish_odom();
        publish_odom_tracking();
        // broadcast_tf();
        // broadcast_tf_map();
    }





    // ===================================== Speed limits ===================================================

    void compute_limited_speed(geometry_msgs::msg::Twist::SharedPtr limited_twist) {
        // Limit acceleration
        if(enable_accel_limit_) {
            // Limit linear acceleration xy
            double current_speed = sqrt(pow(current_vel_.x(), 2) + pow(current_vel_.y(), 2));
            double goal_speed = sqrt(pow(latest_twist_->linear.x, 2) + pow(latest_twist_->linear.y, 2));
            if(goal_speed > max_speed_) {
                goal_speed = max_speed_;
            }
            double cmd_vxy_limited = limit_accel_decel(current_speed, goal_speed, max_accel_linear_, max_decel_linear_, dt_measured_);

            double vel_vect_angle;
            if (goal_speed == 0) {
                // Use angle of the current vel of the robot. This is to avoid the robot to go forward when the goal speed is 0 but the robot is still moving
                vel_vect_angle = atan2(current_vel_.y(), current_vel_.x());
            } else {
                vel_vect_angle = atan2(latest_twist_->linear.y, latest_twist_->linear.x);
            }
            limited_twist->linear.x = cmd_vxy_limited * cos(vel_vect_angle);
            limited_twist->linear.y = cmd_vxy_limited * sin(vel_vect_angle);

            // Limit angular acceleration z
            limited_twist->angular.z = limit_accel_decel(current_vel_.theta(), latest_twist_->angular.z, max_accel_angular_, max_decel_angular_, dt_measured_);
        } else {
            limited_twist->linear.x = latest_twist_->linear.x;
            limited_twist->linear.y = latest_twist_->linear.y;
            limited_twist->angular.z = latest_twist_->angular.z;
        }
    }

    // Returns the velocity with limited acceleration and deceleration applied
    static double limit_accel_decel(double current_speed, double goal_speed, double max_acceleration, double max_deceleration, double dt) {
        // Check if we are at constant speed, accelerating or decelerating
        if (goal_speed == current_speed) {
            // We are at constant speed
            return current_speed;
        } else if ((goal_speed > current_speed && current_speed >= 0) || (goal_speed < current_speed && current_speed <= 0)) {
            // We are accelerating
            return limit_accel(current_speed, goal_speed, max_acceleration, dt);
        } else {
            // We are decelerating
            return limit_decel(current_speed, goal_speed, max_deceleration, dt);
        }
    }


    // Returns the velocity with limited acceleration applied
    static double limit_accel(double current_speed, double goal_speed, double max_acceleration, double dt) {
        // check if goal is smaller or greater than current speed
        if (goal_speed > current_speed) {
            // Compute the speed we can reach in dt
            double max_speed = current_speed + max_acceleration * dt;
            // Check if we can reach the goal speed
            if (max_speed < goal_speed) {
                // We can't reach the goal speed
                return max_speed;
            } else {
                // We can reach the goal speed
                return goal_speed;
            }
        } else {
            // Compute the speed we can reach in dt
            double min_speed = current_speed - max_acceleration * dt;
            // Check if we can reach the goal speed
            if (min_speed > goal_speed) {
                // We can't reach the goal speed
                return min_speed;
            } else {
                // We can reach the goal speed
                return goal_speed;
            }
        }
    }

    // Returns the velocity with limited deceleration applied
    static double limit_decel(double current_speed, double goal_speed, double max_deceleration, double dt) {
        // check if goal is smaller or greater than current speed
        if (goal_speed > current_speed) {
            // Compute the speed we can reach in dt
            double max_speed = current_speed + max_deceleration * dt;
            // Check if we can reach the goal speed
            if (max_speed < goal_speed) {
                // We can't reach the goal speed
                return max_speed;
            } else {
                // We can reach the goal speed
                return goal_speed;
            }
        } else {
            // Compute the speed we can reach in dt
            double min_speed = current_speed - max_deceleration * dt;
            // Check if we can reach the goal speed
            if (min_speed > goal_speed) {
                // We can't reach the goal speed
                return min_speed;
            } else {
                // We can reach the goal speed
                return goal_speed;
            }
        }
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