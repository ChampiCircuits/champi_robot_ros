import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='False',
        description='Launch joystick'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='game_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joy'))
    )

    holo_teleop_joy_node = Node(
        package='dev_tools',
        executable='holo_teleop_joy_node.py',
        name='holo_teleop_joy_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        condition=IfCondition(LaunchConfiguration('joy'))
    )

    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        prefix = 'xterm -e', # Workaround, see https://answers.ros.org/question/337885/is-teleop_twist_keyboard-launchable-in-ros2/
        condition=UnlessCondition(LaunchConfiguration('joy'))
    )

    return LaunchDescription([
        joy_arg,
        joy_node,
        holo_teleop_joy_node,
        teleop_keyboard_node
    ])

