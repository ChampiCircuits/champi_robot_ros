import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import TimerAction


def generate_launch_description():

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='True',
        description='Launch joystick'
    )

    joy_node = TimerAction(
        period=1.0, # s. Wait for 5 seconds to allow the system to boot up and discover the dongle
        actions=[ # en fait ca marche paaaaaaas, donc j'ai mis à 1s
            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='game_controller_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('joy'))
            )
        ]
    )

    holo_teleop_joy_node = Node(
        package='champi_tools',
        executable='holo_teleop_joy_node.py',
        name='holo_teleop_joy_node',
        output='screen',
        parameters=[config_file_path],
        condition=IfCondition(LaunchConfiguration('joy'))
    )

    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/teleop/cmd_vel')],
        prefix = 'xterm -e', # Workaround, see https://answers.ros.org/question/337885/is-teleop_twist_keyboard-launchable-in-ros2/
        condition=UnlessCondition(LaunchConfiguration('joy'))
    )

    return LaunchDescription([
        joy_arg,
        joy_node,
        holo_teleop_joy_node,
        teleop_keyboard_node
    ])

