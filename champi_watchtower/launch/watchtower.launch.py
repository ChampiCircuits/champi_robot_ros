from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config_file_path = os.path.join(get_package_share_directory('champi_watchtower'), 'config', 'watchtower.config.yaml')

    is_simu_with_webots = DeclareLaunchArgument(
        'is_simu_with_webots',
        default_value='False',
        description='Is watchtower launched in simu with webots ?',
    )
    is_simu_with_webots_config = LaunchConfiguration('is_simu_with_webots')

    # Watchtower node
    watchtower_node = Node(
        package='champi_watchtower',
        executable='watchtower_node.py',
        name='watchtower_node',
        output='screen',
        parameters=[
            config_file_path,
            {'is_simu_with_webots': is_simu_with_webots_config}
        ]
    )
    
    return LaunchDescription([
        watchtower_node,
        is_simu_with_webots
    ])
