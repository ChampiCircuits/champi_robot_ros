import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    hardware_interface_node = Node(
        package='champi_controllers',
        executable='hardware_interface_node',
        name='hardware_interface',
        output='screen',
        respawn=True,
        parameters=[
            config_file_path,
        ]
    )

    return LaunchDescription([
        hardware_interface_node
    ])
