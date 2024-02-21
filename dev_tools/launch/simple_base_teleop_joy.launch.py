import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    joy_node = Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            # namespace="dev_tools",
            output='screen',
            # parameters=[os.path.join(get_package_share_directory("fusion_localization"), 'config', 'navsat_transform.cfg.yaml')],
            # remappings=[
            #     ('gps/fix', 'internal/gps/navsat'),
            #     ('imu', 'internal/imu/data')
            # ],
    )

    holo_teleop_joy_node = Node(
            package='dev_tools',
            executable='holo_teleop_joy_node.py',
            name='holo_teleop_joy_node',
            output='screen',
    )

    simple_holo_base_control_node = Node(
            package='dev_tools',
            executable='simple_holo_base_control_node',
            name='simple_holo_base_control_node',
            output='screen',
    )

    return LaunchDescription([
        # joy_node,
        holo_teleop_joy_node,
        simple_holo_base_control_node,
    ])