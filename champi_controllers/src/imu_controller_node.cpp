#include <string>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"

#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"
#include "champi_can/can_ids.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#define CONFIG_RETRY_DELAY 1.0 // If no ret config reveived for this amount of time (s), we retry to send config

using namespace std;

class ImuControllerNode : public rclcpp::Node
{
public:
    ImuControllerNode() :
            Node("imu_node"),
            diag_updater_stm_(this),
            diag_updater_node_(this),
            champi_can_interface_(
                    this->declare_parameter<std::string>("can_interface_name"),
                    {can_ids::IMU_DATA, can_ids::IMU_STATUS},
                    this->declare_parameter<bool>("champi_can_verbose_mode"))
    {

        // Get parameters
        string can_interface_name = this->get_parameter("can_interface_name").as_string();
        timeout_connexion_stm_ = this->declare_parameter<double>("timeout_connexion_stm");
        // Get covariances
        cov_imu_angular_vel_ = this->declare_parameter<std::vector<double>>("imu_covariances.angular_vel");
        // Check if the covariance is the right size (3 values = the diagonal of the covariance matrix)
        if (cov_imu_angular_vel_.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "imu_covariances.angular_vel must have 3 values (diagonal of the covariance matrix). Exiting.");
            return;
        }
        cov_imu_linear_acc_ = this->declare_parameter<std::vector<double>>("imu_covariances.linear_acc");
        // Check if the covariance is the right size (3 values = the diagonal of the covariance matrix)
        if (cov_imu_linear_acc_.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "imu_covariances.linear_acc must have 3 values (diagonal of the covariance matrix). Exiting.");
            return;
        }

        // Set up diagnostics
        diag_updater_stm_.setHardwareID("imu_board");
        diag_updater_stm_.add("stm_status", this, &ImuControllerNode::test_stm_status);

        diag_updater_node_.setHardwareID("none");
        diag_updater_node_.add("node_state", this, &ImuControllerNode::test_node_state);


        // Start the CAN interface
        int ret = champi_can_interface_.start();

        if(ret == 0) {
            RCLCPP_INFO(this->get_logger(), "CAN interface started successfully");

        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Error starting CAN interface");
            exit(1);// TODO handle error
        }

        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Initializing (waiting for first status and/or IMU data)");

        // Reset the stm TODO normalement pas besoin
//        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Resetting the IMU board");
//        reset_stm();

        // Wait a little to let the stm reset
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Create Publishers
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&ImuControllerNode::loop_callback, this));

    }

private:


    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Diagnostic
    diagnostic_updater::Updater diag_updater_stm_;
    diagnostic_updater::Updater diag_updater_node_;

    rclcpp::Time last_rx_status_time_;
    rclcpp::Time last_rx_imu_time_;
    msgs_can::Status current_status_;
    double dt_measured_{};

    bool timeout_connexion_stm_exceeded_ = false;
    bool got_first_imu_data_ = false;
    bool got_first_status_ = false;
    // Structure to store the state of the node. Matches the ROS2 diagnostic msg to be able to publish it easily.
    struct NodeState {
        unsigned char lvl;
        std::string summary;
    } node_state_{};

    // Other variables
    ChampiCan champi_can_interface_;
    msgs_can::ImuData latest_imu_data_;
    struct Pose {
        double x;
        double y;
        double theta;
    } current_pose_{};

    // Parameters
    double timeout_connexion_stm_; // receive status from CAN
    std::vector<double> cov_imu_angular_vel_;
    std::vector<double> cov_imu_linear_acc_;




    // =========================== Diagnostic tests (observation only, for /diagnostics) ===============================

    void test_stm_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        // Summary
        if(!got_first_status_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No status received yet");
        }
        else if(timeout_connexion_stm_exceeded_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Timeout connexion to stm exceeded");
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Timeout connexion to stm exceeded");
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
            stat.add("stm_status", msgs_can::Status_StatusType_Name(current_status_.status()));
        }
        if(current_status_.has_error()) {
            stat.add("stm_error", msgs_can::Status_ErrorType_Name(current_status_.error()));
        }
    }

    void test_node_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        stat.summary(node_state_.lvl, node_state_.summary);
        stat.add("current_loop_rate (Hz)", 1.0 / dt_measured_);
        stat.add("got_first_status", got_first_status_);
        stat.add("got_first_imu_data", got_first_imu_data_);
        stat.add("timeout_connexion_stm_exceeded", timeout_connexion_stm_exceeded_);
    }






    // ======================== Real diagnostics, that can change the state of the node ================================

    void update_node_state(unsigned char lvl, const std::string& summary) {
        node_state_.lvl = lvl;
        node_state_.summary = summary;
        // Print with the correct color
        switch(lvl) {
            case diagnostic_msgs::msg::DiagnosticStatus::OK:
                RCLCPP_INFO(this->get_logger(), "%s", summary.c_str());
                break;
            case diagnostic_msgs::msg::DiagnosticStatus::WARN:
                RCLCPP_WARN(this->get_logger(), "%s", summary.c_str());
                break;
            case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
                RCLCPP_ERROR(this->get_logger(), "%s", summary.c_str());
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
        if(!got_first_status_) {
            return false;
        }

        auto now = this->now();

        // Check if timeout for connexion to the stm is exceeded. If yes, set node state to ERROR and return false.
        timeout_connexion_stm_exceeded_ = now - last_rx_status_time_ > rclcpp::Duration::from_seconds(timeout_connexion_stm_);
        if(timeout_connexion_stm_exceeded_) {
            // Don't do anything to the node state but indicate the error.
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Timeout error (CAN bus). Elapsed time since last status: " + std::to_string((now - last_rx_status_time_).seconds()) + "s");
            diag_updater_node_.force_update(); // Send diagnostic right away
            return true;
        }

        // check if stm status ok
        if(current_status_.has_status() && current_status_.status() != msgs_can::Status_StatusType::Status_StatusType_OK) {
            // Stm not OK. Set node state to ERROR
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "STM status not OK");
            diag_updater_node_.force_update();
            return false;
        }

        // Everything is OK

        if(node_state_.lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN) { // WARN is during initialization TODO pas bien si on veut utilier WARN pour autre chose aussi
            update_node_state(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
            diag_updater_node_.force_update();
        }
        return got_first_imu_data_; // If we have not received the first imu data msg yet, we do not publish it.
    }





    // ================================ Communication with the stm (CAN bus) ==========================================

    void reset_stm() {
        // Send message
        if(champi_can_interface_.send_raw(can_ids::BASE_RESET, "") != 0) { // TODO créer l'ID si on veut utiliser cette fonction
            RCLCPP_ERROR(this->get_logger(), "Error sending reset message");
            // TODO send diagnostic
            exit(1);
        }
    }

    void rx_can() {
        if(champi_can_interface_.check_if_new_full_msg(can_ids::IMU_DATA)) {

            auto buffer = champi_can_interface_.get_full_msg(can_ids::IMU_DATA);

            // Update current imu data
            latest_imu_data_.ParseFromString(buffer);

            last_rx_imu_time_ = this->now(); // TODO for now not used. Compute rx freq for diagnostics

            // flag
            if(!got_first_imu_data_) {
                got_first_imu_data_ = true;
            }
        }
        if(champi_can_interface_.check_if_new_full_msg(can_ids::IMU_STATUS)) {
            last_rx_status_time_ = this->now();
            if(!got_first_status_) {
                got_first_status_ = true;
            }
            auto buffer = champi_can_interface_.get_full_msg(can_ids::IMU_STATUS);
            current_status_.ParseFromString(buffer); // Update current_status_
        }
    }

    //  =========================================== Communication ROS2 =================================================

    void publish_imu() {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // We get gyro and accel data from the IMU board
        msg.angular_velocity.x = latest_imu_data_.gyro_x();
        msg.angular_velocity.y = latest_imu_data_.gyro_y();
        msg.angular_velocity.z = latest_imu_data_.gyro_z();

        msg.linear_acceleration.x = latest_imu_data_.acc_x();
        msg.linear_acceleration.y = latest_imu_data_.acc_y();
        msg.linear_acceleration.z = latest_imu_data_.acc_z();

        // Set covariances
        msg.orientation_covariance[0] = -1; // Orientation is not provided
        msg.orientation_covariance[4] = -1;
        msg.orientation_covariance[8] = -1;

        msg.angular_velocity_covariance[0] = cov_imu_angular_vel_[0];
        msg.angular_velocity_covariance[4] = cov_imu_angular_vel_[1];
        msg.angular_velocity_covariance[8] = cov_imu_angular_vel_[2];

        msg.linear_acceleration_covariance[0] = cov_imu_linear_acc_[0];
        msg.linear_acceleration_covariance[4] = cov_imu_linear_acc_[1];
        msg.linear_acceleration_covariance[8] = cov_imu_linear_acc_[2];

        pub_imu_->publish(msg);
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

        // Read incoming message if available
        rx_can();

        // - Initialization is not finished ?
        // - the node is OK or WARN ?
        bool node_ok = check_node_ok_and_update();

        if(!node_ok) {
            return;
        }
        // Publish IMU data
        publish_imu();
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuControllerNode>());
    rclcpp::shutdown();
    return 0;
}