from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    planner_node = Node(
        package='champi_navigation',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
    )

    pose_control_node = Node(
        package='champi_navigation',
        executable='path_controller.py',
        name='path_controller',
        output='screen',
    )

    costmap_updater_node = Node(
        package='champi_navigation',
        executable='costmap_updater.py',
        name='costmap_updater',
        output='screen',
    )


    return LaunchDescription([
        # planner_node,
        # pose_control_node
        costmap_updater_node
    ])