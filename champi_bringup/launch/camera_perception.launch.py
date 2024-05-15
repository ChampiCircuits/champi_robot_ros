import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')


    raspi_cam_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_vision'),
            '/launch/raspi_cam.launch.py'
        ])
    )


    visual_loc = Node(
            package="champi_vision",
            executable="visual_loc_node.py",
            name='visual_loc',
            output='screen'
    )


    

    return LaunchDescription([
        raspi_cam_launch,
        visual_loc
    ])

