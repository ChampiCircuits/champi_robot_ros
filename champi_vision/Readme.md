ros2 run champi_vision platform_detection_node.py 

les deux bags
~/Downloads$ ros2 bag play -l rosbag2_tf
/Downloads$ ros2 bag play -l rosbag2_2025_02_23-00_44_53



#  pour tester la cam
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true

# bring up
ros2 launch champi_bringup bringup.launch.py sim:=True nav:=True

# Cam info for bird view :
ros2 run champi_vision camera_info_publisher_node.py --ros-args --param calib_yaml_path:=/home/sebastien/ws_champi/src/champi_robot_ros/champi_vision/config/calib/champi_cam.yaml 

# publish tranform
ros2 run tf2_ros static_transform_publisher 0. 0. 0. 0.9 0. 0. 1 camera_link map
ros2 run tf2_ros static_transform_publisher 0.15 0. 0.31 -2.10 0.0 0.0 1 camera_link map
