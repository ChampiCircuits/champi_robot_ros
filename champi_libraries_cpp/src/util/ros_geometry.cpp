#include "util/ros_geometry.h"

#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace easy_geometry {

geometry_msgs::msg::Pose add(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {

    tf2::Transform tf_a, tf_b;
    tf2::fromMsg(a, tf_a);
    tf2::fromMsg(b, tf_b);

    tf2::Transform tf_result = tf_a * tf_b;

    geometry_msgs::msg::Pose result;
    tf2::toMsg(tf_result, result);
    return result;
}

} // namespace easy_geometry