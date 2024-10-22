// sudo apt-get install ros-humble-pcl-conversions ros-humble-pcl-msgs
// sudo apt install libpcl-dev

#include <random>
#include <vector>
#include <cmath>
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "icp_pcl.hpp"

const double R_BEACON = 0.055;

class AbsoluteLocNode : public rclcpp::Node
{
public:
    AbsoluteLocNode() : Node("absolute_loc")
    {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AbsoluteLocNode::laser_callback, this, std::placeholders::_1));

        last_scan_ = nullptr;

        ref_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ref_point_cloud", 10);
        current_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/current_point_cloud", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&AbsoluteLocNode::timer_callback, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Absolute loc node launched !");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_point_cloud_pub_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
        std::cout << "new scan" << std::endl;
    }

    void timer_callback()
    {
        if (!last_scan_)
        {
            return;
        }
        compute();
    }

    void compute()
    {
        // Générer les cercles pour les balises
        auto ref_point_cloud = generate_pointcloud_with_circles(R_BEACON * 2, {
            create_point_stamped(0.8, 1.5),
            create_point_stamped(2.0, 1.5),
            create_point_stamped(2.0, 0.8)
        });

        auto point_cloud_in_odom = transform_scan_in_odom_point_cloud(last_scan_);

        if (ref_point_cloud.data.empty() || point_cloud_in_odom.data.empty())
        {
            return;
        }

        // Appliquer l'ICP ici
        auto transformation_matrix = applyICPFromROSPointClouds(point_cloud_in_odom, ref_point_cloud);

        std::cout << transformation_matrix << std::endl;

        ref_point_cloud_pub_->publish(ref_point_cloud);
        current_point_cloud_pub_->publish(point_cloud_in_odom);

        std::cout << "loop" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 generate_pointcloud_with_circles(double diameter, const std::vector<geometry_msgs::msg::PointStamped> &positions, int num_points = 60, double uniform = 0.0)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "odom";

        // Définir les champs de nuage de points
        sensor_msgs::msg::PointField x_field, y_field, z_field;
        x_field.name = "x"; x_field.offset = 0; x_field.datatype = sensor_msgs::msg::PointField::FLOAT32; x_field.count = 1;
        y_field.name = "y"; y_field.offset = 4; y_field.datatype = sensor_msgs::msg::PointField::FLOAT32; y_field.count = 1;
        z_field.name = "z"; z_field.offset = 8; z_field.datatype = sensor_msgs::msg::PointField::FLOAT32; z_field.count = 1;

        cloud.fields.push_back(x_field);
        cloud.fields.push_back(y_field);
        cloud.fields.push_back(z_field);

        cloud.point_step = 12; // 3 * 4 bytes (float32)
        cloud.row_step = 0; // sera mis à jour lors de la publication
        cloud.is_dense = true;

        std::vector<float> points;
        double radius = diameter / 2.0;

        for (const auto &position : positions)
        {
            double center_x = position.point.x;
            double center_y = position.point.y;
            int num_circle_points = num_points / positions.size();

            for (int i = 0; i < num_circle_points; ++i)
            {
                double angle = 2 * M_PI * i / num_circle_points;
                points.push_back(center_x + radius * std::cos(angle)); // x
                points.push_back(center_y + radius * std::sin(angle)); // y
                points.push_back(0.0); // z
            }
        }

        // Copier les points dans le nuage de points
        cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(points.data()), reinterpret_cast<const uint8_t*>(points.data()) + points.size() * sizeof(float));
        cloud.height = 1; // nuage de points non structuré
        cloud.width = points.size() / 3; // nombre total de points

        return cloud;
    }

    sensor_msgs::msg::PointCloud2 transform_scan_in_odom_point_cloud(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "odom";

        // Définir les champs de nuage de points
        sensor_msgs::msg::PointField x_field, y_field, z_field;
        x_field.name = "x"; x_field.offset = 0; x_field.datatype = sensor_msgs::msg::PointField::FLOAT32; x_field.count = 1;
        y_field.name = "y"; y_field.offset = 4; y_field.datatype = sensor_msgs::msg::PointField::FLOAT32; y_field.count = 1;
        z_field.name = "z"; z_field.offset = 8; z_field.datatype = sensor_msgs::msg::PointField::FLOAT32; z_field.count = 1;

        cloud.fields.push_back(x_field);
        cloud.fields.push_back(y_field);
        cloud.fields.push_back(z_field);

        cloud.point_step = 12; // 3 * 4 bytes (float32)
        cloud.row_step = 0; // sera mis à jour lors de la publication
        cloud.is_dense = true;

        double angle = laser_scan->angle_min;
        std::vector<float> points;

        for (auto r : laser_scan->ranges)
        {
            if (laser_scan->range_min < r && r < laser_scan->range_max)
            {
                geometry_msgs::msg::PointStamped point_in_scan;
                point_in_scan.header.frame_id = laser_scan->header.frame_id;
                point_in_scan.point.x = r * std::cos(angle);
                point_in_scan.point.y = r * std::sin(angle);
                point_in_scan.point.z = 0.0;

                geometry_msgs::msg::PointStamped point_in_odom;
                try
                {
                    point_in_odom = tf_buffer_->transform(point_in_scan, "odom");
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
                    return cloud;
                }

                // Ajouter les coordonnées du point au nuage
                points.push_back(point_in_odom.point.x);
                points.push_back(point_in_odom.point.y);
                points.push_back(0.0); // z
            }
            angle += laser_scan->angle_increment;
        }

        // Copier les points dans le nuage de points
        cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(points.data()), reinterpret_cast<const uint8_t*>(points.data()) + points.size() * sizeof(float));
        cloud.height = 1; // nuage de points non structuré
        cloud.width = points.size() / 3; // nombre total de points

        return cloud;
    }

    geometry_msgs::msg::PointStamped create_point_stamped(double x, double y)
    {
        geometry_msgs::msg::PointStamped point;
        point.point.x = x;
        point.point.y = y;
        point.point.z = 0.0;
        return point;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AbsoluteLocNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
