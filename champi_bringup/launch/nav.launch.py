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

    a_star_path_planner_node = Node(
        package='champi_navigation',
        executable='a_star_path_planner.py',
        name='path_planner',
        output='screen',
    )

    call_set_pose_node = Node(
        package='dev_tools',
        executable='call_set_pose.py',
        name='call_set_pose',
        output='screen',
    )


    return LaunchDescription([
        # planner_node,
        # pose_control_node
        costmap_updater_node,
        a_star_path_planner_node,
        call_set_pose_node
    ])