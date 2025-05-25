import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # Declare the launch options
    sim_arg = DeclareLaunchArgument(
        'sim',
        description='Launch simulation (true|false)',
    )

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    lidar_simu_node = Node(
        package='champi_simulator',
        executable='simu_lidar_node.py',
        name='lidar_simu',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    enemy_tracker_node = Node(
        package='champi_vision',
        executable='enemy_tracker_node.py',
        name='enemy_tracker',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        sim_arg,
        lidar_simu_node,
        enemy_tracker_node
    ])

