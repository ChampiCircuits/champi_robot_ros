import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


"""
Ce launch file lance les nodes liés à la base et à la localisation du robot.
"""

def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    # https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/README.md
    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={'pointcloud.enable': 'true'}.items()
    )

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld_lidar',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        realsense2_camera_launch,
        ldlidar_node
    ])

