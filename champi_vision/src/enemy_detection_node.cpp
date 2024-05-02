#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


using namespace std;


class EnemyDetectionNode : public rclcpp::Node
{
public:
    EnemyDetectionNode() : Node("enemy_detection_node") {


        // ================================ Get parameters =================================

        double loop_freq = this->declare_parameter<double>("loop_freq", 10.0);


        // ================================ Initialize ROS related ===================================

        // Subscriber for laser scan TODO QoS
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&EnemyDetectionNode::laser_scan_callback, this, std::placeholders::_1)
        );

        // Timer for main loop
        loop_timer_ = this->create_wall_timer(
            1s/loop_freq, std::bind(&EnemyDetectionNode::loop_callback, this)
        );

        // TF related
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    }

private:

    // Subscriber for laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr loop_timer_;

    // TF related
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;


    // ================================ Callbacks ===================================

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received laser scan");
    }

    void loop_callback() {

    }

    // ================================ Detection ===================================


    void detect_enemy(sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {

        // Iterate over the laser scan and compute point for each
        for (int i = 0; i < laser_scan->ranges.size(); i++) {
            double range = laser_scan->ranges[i];
            double angle = laser_scan->angle_min + i * laser_scan->angle_increment;
            double x = range * cos(angle);
            double y = range * sin(angle);
        }
    }


    // ================================ Utils ===================================

    void get_transform_scan_to_map(sensor_msgs::msg::LaserScan::SharedPtr laser_scan, geometry_msgs::msg::TransformStamped &transform) {
        try {
            transform = tf_buffer_->lookupTransform("map", laser_scan->header.frame_id, laser_scan->header.stamp);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    void transform_point(double x, double y, geometry_msgs::msg::TransformStamped &transform) {
        double x_new = transform.transform.translation.x + x * cos(transform.transform.rotation.z) - y * sin(transform.transform.rotation.z);
        double y_new = transform.transform.translation.y + x * sin(transform.transform.rotation.z) + y * cos(transform.transform.rotation.


    }



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnemyDetectionNode>());
    rclcpp::shutdown();
    return 0;
}