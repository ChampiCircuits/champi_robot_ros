import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')


# https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/README.md
    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={'pointcloud.enable': 'true'}.items()
    )

    visual_loc = Node(
            package="champi_vision",
            executable="aruco_localizer_node.py",
            name='aruco_loc',
            output='screen'
    )

    return LaunchDescription([
        realsense2_camera_launch,
        visual_loc
    ])

