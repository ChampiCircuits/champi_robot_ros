#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "champi_can/msgs_can.pb.h"
#include "champi_can/champi_can.hpp"
#include "champi_can/can_ids.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"


#include <diagnostic_updater/diagnostic_updater.hpp>

#define CONFIG_RETRY_DELAY 1.0 // If no ret config reveived for this amount of time (s), we retry to send config

using namespace std;

class ActuatorsControllerNode : public rclcpp::Node
{
public:
    ActuatorsControllerNode() :
            Node("actuators_node"),
            diag_updater_stm_(this),
            diag_updater_node_(this),
            champi_can_interface_(
                    this->declare_parameter<std::string>("can_interface_name"),
                    {can_ids::ACT_STATUS, can_ids::LASERS_DISTANCES},
                    this->declare_parameter<bool>("champi_can_verbose_mode"))
    {

        // Get parameters
        string can_interface_name = this->get_parameter("can_interface_name").as_string();

        // Set up diagnostics
        diag_updater_stm_.setHardwareID("act_board");
        diag_updater_stm_.add("stm_status", this, &ActuatorsControllerNode::test_stm_status);

        diag_updater_node_.setHardwareID("none");
        diag_updater_node_.add("node_state", this, &ActuatorsControllerNode::test_node_state);


        // Start the CAN interface
        int ret = champi_can_interface_.start();

        if(ret == 0) {
            RCLCPP_INFO(this->get_logger(), "CAN interface started successfully");

        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Error starting CAN interface");
            exit(1);// TODO handle error
        }

        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Initializing (waiting for first status and/or other data from stm)");

        // Reset the stm
        update_node_state(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Resetting the Act board");
        reset_stm();

        // Wait a little to let the stm reset
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Create Publishers
        pub_tirette_start_ = this->create_publisher<std_msgs::msg::Empty>("/tirette_start", 10);
        pub_act_status_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/act_status", 10);

        // pub_lasers_

        // Sub
        sub_cmd_= this->create_subscription<std_msgs::msg::Int64>("/act_cmd", 10, std::bind(
                &ActuatorsControllerNode::act_cmd_callback, this, std::placeholders::_1));

        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&ActuatorsControllerNode::loop_callback, this));

    }

private:


    // Publishers: Pub current action and pub tirette start
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_tirette_start_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_act_status_;


    // Subscriber for cmd Action
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_cmd_;


    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Diagnostic
    diagnostic_updater::Updater diag_updater_stm_;
    diagnostic_updater::Updater diag_updater_node_;

    msgs_can::ActStatus latest_act_status_;
    msgs_can::LasersDistances latest_lasers_distances_;

    rclcpp::Time last_rx_status_time_;
    double dt_measured_{};

    bool got_first_act_status_ = false;
    // Structure to store the state of the node. Matches the ROS2 diagnostic msg to be able to publish it easily.
    struct NodeState {
        unsigned char lvl;
        std::string summary;
    } node_state_{};

    // Other variables
    ChampiCan champi_can_interface_;


    // =========================== Diagnostic tests (observation only, for /diagnostics) ===============================

    void test_stm_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        // Summary
        if(!got_first_act_status_) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No act status received yet");
        }
        else if(latest_act_status_.has_status() && latest_act_status_.status().has_status()) {
            switch(latest_act_status_.status().status()) {
                case msgs_can::Status_StatusType::Status_StatusType_OK:
                    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK2");
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
        if(latest_act_status_.has_status() && latest_act_status_.status().has_status()) {
            stat.add("stm_status", msgs_can::Status_StatusType_Name(latest_act_status_.status().status()));
        }
        if(latest_act_status_.has_status() && latest_act_status_.status().has_error()) {
            stat.add("stm_error", msgs_can::Status_ErrorType_Name(latest_act_status_.status().error()));
        }
    }

    void test_node_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        stat.summary(node_state_.lvl, node_state_.summary);
        stat.add("current_loop_rate (Hz)", 1.0 / dt_measured_);
        stat.add("got_first_act_status", got_first_act_status_);
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
        if(!got_first_act_status_) {
            return false;
        }

        auto now = this->now();

        // check if stm status ok
        if(latest_act_status_.has_status() && latest_act_status_.status().status() != msgs_can::Status_StatusType::Status_StatusType_OK) {
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
        return got_first_act_status_; // If we have not received the first imu data msg yet, we do not publish it.
    }





    // ================================ Communication with the stm (CAN bus) ==========================================

    void reset_stm() {
        // Send message
        if(champi_can_interface_.send_raw(can_ids::ACT_RESET, "") != 0) { // TODO créer l'ID si on veut utiliser cette fonction
            RCLCPP_ERROR(this->get_logger(), "Error sending reset message");
            // TODO send diagnostic
            exit(1);
        }
    }

    void rx_can() {
        if(champi_can_interface_.check_if_new_full_msg(can_ids::ACT_STATUS)) {
            // TODO il devrait pas y avoir un             last_rx_status_time_ = this->now(); ici aussi ?
            auto buffer = champi_can_interface_.get_full_msg(can_ids::ACT_STATUS);

            // Update current imu data
            latest_act_status_.ParseFromString(buffer);

            // flag
            if(!got_first_act_status_) {
                got_first_act_status_ = true;
            }


            publish_act_status();
        }

        if(champi_can_interface_.check_if_new_full_msg(can_ids::TIRETTE_START)) {
            last_rx_status_time_ = this->now();
            auto buffer = champi_can_interface_.get_full_msg(can_ids::TIRETTE_START);
            
            publish_tirette_start();
        }


        if(champi_can_interface_.check_if_new_full_msg(can_ids::LASERS_DISTANCES)) {

            last_rx_status_time_ = this->now();
            auto buffer = champi_can_interface_.get_full_msg(can_ids::LASERS_DISTANCES);
            RCLCPP_INFO(this->get_logger(), "SIZE buffer : %d", buffer.size()); 

            // for (auto c:buffer)
            //     RCLCPP_WARN(this->get_logger(), "%d", (int) c); 

            // Update current lasers_distances data
            latest_lasers_distances_.ParseFromString(buffer);

            RCLCPP_INFO(this->get_logger(), "SIZE : %d",latest_lasers_distances_.distances_size()); 


            for (int i=0;i<4;i++) {
              RCLCPP_INFO(this->get_logger(), "%f",latest_lasers_distances_.distances(i)); 
            }

            // // publish_lasers_distances();
        }
    }

    //  =========================================== Communication ROS2 =================================================

    
    void act_cmd_callback(const std_msgs::msg::Int64::SharedPtr msg) {

        msgs_can::ActCmd act_cmd;

        if(msg->data==0) {
            act_cmd.set_action(msgs_can::ActActions::START_GRAB_PLANTS);
        }
        else if(msg->data==1) {
            act_cmd.set_action(msgs_can::ActActions::STOP_GRAB_PLANTS);
        }
        else if(msg->data==2) {
            act_cmd.set_action(msgs_can::ActActions::RELEASE_PLANT);
        }
        else if(msg->data==3) {
            act_cmd.set_action(msgs_can::ActActions::TURN_SOLAR_PANEL);
        }
        else if(msg->data==4) {
            act_cmd.set_action(msgs_can::ActActions::INITIALIZING);
        }
        else if(msg->data==5) {
            act_cmd.set_action(msgs_can::ActActions::FREE);
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Issue: whe should never reach this else :/");
        }

        // Send message
        if(champi_can_interface_.send(can_ids::ACT_ACTION, act_cmd.SerializeAsString()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error sending message");
            // TODO send diagnostic
        }
        
    }
    
    
    void publish_act_status() {
        std_msgs::msg::Int64MultiArray current_status;
        
        int status;

        switch(latest_act_status_.action()) {
            case msgs_can::ActActions::START_GRAB_PLANTS:
                status = 0;
                break;
            case msgs_can::ActActions::STOP_GRAB_PLANTS:
                status = 1;
                break;
            case msgs_can::ActActions::RELEASE_PLANT:
                status = 2;
                break;
            case msgs_can::ActActions::TURN_SOLAR_PANEL:
                status = 3;
                break;
            case msgs_can::ActActions::INITIALIZING:
                status = 4;
                break;
            case msgs_can::ActActions::FREE:
                status = 5;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Issue: whe should never reach this print :/");
            
        }

        current_status.data = std::vector<int64_t>({status, latest_act_status_.plant_count()});


        pub_act_status_->publish(current_status);
    }

    void publish_lasers_distances() {
        std_msgs::msg::Int64MultiArray lasers_distances;

        RCLCPP_INFO(this->get_logger(), "RECEIVED LASER DISTANCES !! %d \t %d \t %d \t %d", 
            (int)latest_lasers_distances_.distances(0),
            (int)latest_lasers_distances_.distances(1),
            (int)latest_lasers_distances_.distances(2),
            (int)latest_lasers_distances_.distances(3));

        lasers_distances.data = std::vector<int64_t>({(int)latest_lasers_distances_.distances(0),
                                                    (int)latest_lasers_distances_.distances(1),
                                                    (int)latest_lasers_distances_.distances(2),
                                                    (int)latest_lasers_distances_.distances(3)});
        
        // publish_lasers_distances.publish(lasers_distances);

    }


    void publish_tirette_start() {
        std_msgs::msg::Empty tirette_start;
        pub_tirette_start_->publish(tirette_start);
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

    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorsControllerNode>());
    rclcpp::shutdown();
    return 0;
}