#include <string>

#include <rclcpp/rclcpp.hpp>


#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

using namespace std;

class BaseControllerNode : public rclcpp::Node
{
public:
    BaseControllerNode() :
            Node("test"),
            diag_updater_base_(this),
            updater2_(this)
    {

        // Set up diagnostics
        diag_updater_base_.setHardwareID("holo_base");
        diag_updater_base_.add("diag_counter", this, &BaseControllerNode::test_base_status);

        updater2_.setHardwareID("node");
        updater2_.add("diag_counter", this, &BaseControllerNode::test_node_state);


        // Timer loop
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&BaseControllerNode::loop_callback, this));

    }

private:

    int cnt = 0;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Diagnostic
    //double pub_min_freq_;
    //double pub_max_freq_;
    diagnostic_updater::Updater diag_updater_base_;
    diagnostic_updater::Updater updater2_;
    //diagnostic_updater::HeaderlessTopicDiagnostic pub_diag_;

    // Functions

    void test_base_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Everything is fine.");
        stat.add("counter_value", cnt);
    }

    void test_node_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Everything is fine too.");
        stat.add("counter_value2", cnt*2);
    }


    void loop_callback() {

        cnt += 1;

    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseControllerNode>());
    rclcpp::shutdown();
    return 0;
}