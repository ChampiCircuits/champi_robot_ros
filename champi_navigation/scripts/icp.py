import numpy as np
import math
import sensor_msgs.msg
import simpleicp
import icp_lib
import matplotlib.pyplot as plt

def apply_icp_from_ros_pointclouds(source_cloud_ros: sensor_msgs.msg.PointCloud, target_cloud_ros: sensor_msgs.msg.PointCloud):
    """
    Applique l'algorithme ICP pour aligner deux nuages de points ROS 2 de type PointCloud.
    
    :param source_cloud_ros: Nuage de points source de type sensor_msgs.msg.PointCloud (ROS 2)
    :param target_cloud_ros: Nuage de points cible de type sensor_msgs.msg.PointCloud (ROS 2)
    :return: transformation_matrix: Matrice de transformation qui aligne le nuage source sur le nuage cible
    """

    # Convertir le nuage de points ROS en tableaux NumPy
    def ros_pointcloud_to_numpy(pointcloud: sensor_msgs.msg.PointCloud):
        points = []
        for point in pointcloud.points:
            points.append([point.x, point.y])#, point.z])
        return np.array(points)

    # Conversion des deux nuages de points ROS en format numpy
    source_points_np = ros_pointcloud_to_numpy(source_cloud_ros)
    target_points_np = ros_pointcloud_to_numpy(target_cloud_ros)

    print(source_points_np)
    print()
    print()
    print()
    print(target_points_np)

    if source_points_np.shape[0] == 0 or target_points_np.shape[0] == 0:
        raise ValueError("Source or target point cloud is empty or contains invalid points.")

    transformation_history, aligned_points = icp_lib.icp(source_points_np, target_points_np, verbose=True)
    reference_points = source_points_np
    points_to_be_aligned = target_points_np
    transformation_history = np.array(transformation_history)
    #transformation_history (theta, x, y)
    # print(transformation_history)
    # print(transformation_history[:,1])
    # print(np.sum(transformation_history[:,2]))


    plt.plot(reference_points[:, 0], reference_points[:, 1], 'rx', label='reference points')
    plt.plot(points_to_be_aligned[:, 0], points_to_be_aligned[:, 1], 'b1', label='points to be aligned')
    plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
    plt.legend()
    plt.show()
    # Créer des objets PointCloud pour SimpleICP
    # source_cloud_icp = simpleicp.PointCloud(source_points_np, columns=["x", "y", "z"])
    # target_cloud_icp = simpleicp.PointCloud(target_points_np, columns=["x", "y", "z"])

    # Créer l'objet SimpleICP
    # icp = simpleicp.SimpleICP()

    # Ajouter les nuages de points à l'ICP
    # icp.add_point_clouds(target_cloud_icp, source_cloud_icp)
    
    # Appliquer l'algorithme ICP
    # H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1.0)

    # H est la matrice de transformation (4x4) qui aligne le nuage source avec le nuage cible
    # return H, X_mov_transformed, rigid_body_transformation_params, distance_residuals