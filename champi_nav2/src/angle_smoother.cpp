// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <vector>
#include <memory>
#include "champi_nav2_plugins/angle_smooher.hpp"

namespace champi_nav2_plugins
{
    using namespace smoother_utils;  // NOLINT
    using namespace nav2_util::geometry_utils;  // NOLINT
    using namespace std::chrono;  // NOLINT
    using nav2_util::declare_parameter_if_not_declared;

    void AngleSmoother::configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
            std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
            std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
    {
        costmap_sub_ = costmap_sub;

        auto node = parent.lock();
        logger_ = node->get_logger();

        declare_parameter_if_not_declared(
                node, name + ".interpolate", rclcpp::ParameterValue(false));

        node->get_parameter(name + ".interpolate", interpolate_);
    }

    bool AngleSmoother::smooth(
            nav_msgs::msg::Path & path,
            const rclcpp::Duration & max_time)
    {

        if(!interpolate_){
            // Set all the orientations to the same as the last one
            for (int i = 0; i < path.poses.size()-1; i++) {
                path.poses[i].pose.orientation = path.poses[path.poses.size() - 1].pose.orientation;
            }
            return true;
        }

        // If the path has less than 3 points, we can't smooth it
        if (path.poses.size() == 1) {
            return true;
        }
        if (path.poses.size() == 2) {
            // set first orientation to the same as the second
            path.poses[0].pose.orientation = path.poses[1].pose.orientation;
            return true;
        }

        int nb_points = path.poses.size();

        tf2::Quaternion q_start, q_end;
        tf2::fromMsg(path.poses[0].pose.orientation, q_start);
        tf2::fromMsg(path.poses[nb_points - 1].pose.orientation, q_end);
        double angle_diff = tf2::angleShortestPath(q_start, q_end);

        // Print the normalized angle_diff
        RCLCPP_INFO(logger_, "Normalized angle diff: %f", angle_diff);


        // Iterate through each point in the path, and recalculate each angle adding step in respect to the angle_diff and the number of points
        for (int i = 1; i < nb_points-1; i++) {
            double angle = tf2::getYaw(path.poses[i].pose.orientation);
            angle += angle_diff * i / nb_points;
            path.poses[i].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));
        }


        return true;
    }

    bool AngleSmoother::smoothImpl(
            nav_msgs::msg::Path & path,
            bool & reversing_segment,
            const nav2_costmap_2d::Costmap2D * costmap,
            const double & max_time)
    {
        // We don't use this method
        return true;
    }

    double AngleSmoother::getFieldByDim(
            const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
    {
        if (dim == 0) {
            return msg.pose.position.x;
        } else if (dim == 1) {
            return msg.pose.position.y;
        } else {
            return msg.pose.position.z;
        }
    }

    void AngleSmoother::setFieldByDim(
            geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
            const double & value)
    {
        if (dim == 0) {
            msg.pose.position.x = value;
        } else if (dim == 1) {
            msg.pose.position.y = value;
        } else {
            msg.pose.position.z = value;
        }
    }

}  // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(champi_nav2_plugins::AngleSmoother, nav2_core::Smoother)