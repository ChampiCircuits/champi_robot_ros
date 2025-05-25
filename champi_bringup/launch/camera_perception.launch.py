import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    visual_loc = Node(
            package="champi_vision",
            executable="aruco_localizer_node.py",
            name='aruco_localizer_node',
            output='screen'
    )

    platform_detection = Node(
        package="champi_vision",
        executable="platform_detection_node.py",
        name='platform_detection',
        output='screen'
    )

    return LaunchDescription([
        visual_loc,
        platform_detection
    ])

