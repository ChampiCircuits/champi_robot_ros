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
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <sensor_msgs/msg/point_field.hpp>

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

sensor_msgs::msg::PointCloud2 getCorrespondencesPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                                                           double max_distance)
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
    cloud.is_dense = true;

    std::vector<float> points;

    // Utiliser CorrespondenceEstimation pour trouver les correspondances
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> correspondence_estimation;
    correspondence_estimation.setInputSource(target_cloud);
    correspondence_estimation.setInputTarget(source_cloud);

    pcl::Correspondences all_correspondences;
    correspondence_estimation.determineCorrespondences(all_correspondences, max_distance);

    // Stocker les points de correspondance dans le nuage
    for (const auto &corr : all_correspondences)
    {
        const auto &target_point = target_cloud->points[corr.index_query];
        const auto &source_point = source_cloud->points[corr.index_match];

        // Ajout du point correspondant dans le nuage de points
        points.push_back(target_point.x);
        points.push_back(target_point.y);
        points.push_back(target_point.z);

        points.push_back(source_point.x);
        points.push_back(source_point.y);
        points.push_back(source_point.z);
    }

    // Copier les données de points dans le nuage de points ROS
    cloud.data.insert(cloud.data.end(), reinterpret_cast<const uint8_t*>(points.data()), 
                      reinterpret_cast<const uint8_t*>(points.data()) + points.size() * sizeof(float));
    cloud.height = 1; // nuage de points non structuré
    cloud.width = points.size() / 3; // nombre total de points

    return cloud;
}


Eigen::Matrix4f applyICPWithCorrespondences(const sensor_msgs::msg::PointCloud2 &source_cloud_ros,
                                            const sensor_msgs::msg::PointCloud2 &target_cloud_ros, 
                                            pcl::PointCloud<pcl::PointXYZ> &aligned_cloud,
                                            sensor_msgs::msg::PointCloud2 &correspondence_cloud) 
{
    // Convertir les nuages de points ROS en nuages de points PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Convertir le nuage source et cible
    pcl::fromROSMsg(source_cloud_ros, *source_cloud);
    pcl::fromROSMsg(target_cloud_ros, *target_cloud);

    // Appliquer l'algorithme ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    icp.setMaximumIterations(200);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setTransformationEpsilon(1e-8);
    icp.setTransformationRotationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setRANSACIterations(10000);
    icp.setRANSACOutlierRejectionThreshold(0.5);
    icp.setUseReciprocalCorrespondences(true);

    icp.align(aligned_cloud);
    // Obtenir la matrice de transformation
    transformation_matrix = icp.getFinalTransformation();

    if (icp.hasConverged()) 
    {
        // Récupérer les correspondances sous forme de PointCloud2
        correspondence_cloud = getCorrespondencesPointCloud(source_cloud, target_cloud, 0.2);
        std::cout << "ICP converged." << std::endl;
    } 
    else 
    {
        std::cout << "ICP did not converge." << std::endl;
    }

    return transformation_matrix;
}

