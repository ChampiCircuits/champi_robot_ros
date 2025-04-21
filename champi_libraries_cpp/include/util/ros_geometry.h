#ifndef ROS_GEOMETRY_HPP
#define ROS_GEOMETRY_HPP

#include <geometry_msgs/msg/pose.hpp>

namespace ros_geometry {

geometry_msgs::msg::Pose add(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b);

geometry_msgs::msg::Pose sub(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b);

geometry_msgs::msg::Pose inverse(const geometry_msgs::msg::Pose& pose);

} // namespace ros_geometry





#endif //ROS_GEOMETRY_HPP
