#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// Fonction pour convertir un sensor_msgs::msg::PointCloud en PCL PointCloud
PointCloudT::Ptr rosPointCloudToPCL(const sensor_msgs::msg::PointCloud& ros_cloud) {
    PointCloudT::Ptr pcl_cloud(new PointCloudT);
    for (const auto& point : ros_cloud.points) {
        pcl_cloud->push_back(PointT(point.x, point.y, point.z));
    }
    return pcl_cloud;
}

// Fonction ICP pour aligner deux nuages de points
Eigen::Matrix4f applyICP(const PointCloudT::Ptr& source_cloud, const PointCloudT::Ptr& target_cloud) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    icp.align(*aligned_cloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
        return icp.getFinalTransformation();
    } else {
        std::cerr << "ICP did not converge." << std::endl;
        return Eigen::Matrix4f::Identity();
    }
}
Eigen::Matrix4f applyICPFromROSPointClouds(const sensor_msgs::msg::PointCloud2 &source_cloud_ros,
                                            const sensor_msgs::msg::PointCloud2 &target_cloud_ros) {
    // Convertir les nuages de points ROS en nuages de points PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Convertir le nuage source
    pcl::fromROSMsg(source_cloud_ros, *source_cloud);
    // Convertir le nuage cible
    pcl::fromROSMsg(target_cloud_ros, *target_cloud);


    // Appliquer l'algorithme ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.align(aligned_cloud);

    // Obtenir la matrice de transformation
    transformation_matrix = icp.getFinalTransformation();

    return transformation_matrix; // Retourne la matrice de transformation
}