from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    pose_controller_node = Node(
        package='champi_navigation',
        executable='pose_controller_node.py',
        name='pose_controller',
        output='screen',
        parameters=[config_file_path]
    )

    costmap_updater_node = Node(
        package='champi_navigation',
        executable='costmap_updater_node.py',
        name='costmap_updater',
        output='screen',
        parameters=[config_file_path]
    )

    a_star_path_planner_node = Node(
        package='champi_navigation',
        executable='path_planner_node.py',
        name='path_planner',
        output='screen',
        parameters=[config_file_path]
    )

    path_planner_ui_node = Node(
        package='champi_navigation',
        executable='path_planner_ui_node.py',
        name='path_planner_ui',
        output='screen',
    )

    return LaunchDescription([
        pose_controller_node,
        # costmap_updater_node,
        a_star_path_planner_node,
        path_planner_ui_node

    ])