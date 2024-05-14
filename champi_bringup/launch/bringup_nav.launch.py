from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    planner_node = Node(
        package='champi_navigation',
        executable='planner_node.py',
        name='planner_node',
        output='screen',
    )

    pose_control_node = Node(
        package='champi_navigation',
        executable='pose_control_node.py',
        name='pose_control_node',
        output='screen',
    )


    return LaunchDescription([
        # planner_node,
        pose_control_node
    ])