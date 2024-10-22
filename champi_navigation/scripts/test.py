from simpleicp import PointCloud, SimpleICP
import numpy as np

# Read point clouds from xyz files into n-by-3 numpy arrays
X_fix = np.genfromtxt("/home/ku-etienne/ros_ws/src/champi_robot_ros/champi_navigation/scripts/1.xyz")
X_mov = np.genfromtxt("/home/ku-etienne/ros_ws/src/champi_robot_ros/champi_navigation/scripts/2.xyz")

# Create point cloud objects
pc_fix = PointCloud(X_fix, columns=["x", "y", "z"])
pc_mov = PointCloud(X_mov, columns=["x", "y", "z"])

# Create simpleICP object, add point clouds, and run algorithm!
icp = SimpleICP()
icp.add_point_clouds(pc_fix, pc_mov)
H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)