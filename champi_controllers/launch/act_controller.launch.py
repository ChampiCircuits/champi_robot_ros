import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare the launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='The CAN interface to use [can0, vcan0]'
    )

    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    act_controller_node = Node(
        package='champi_controllers',
        executable='actuators_controller_node',
        name='act_controller',
        output='screen',
        parameters=[
            config_file_path,
            {'can_interface_name': LaunchConfiguration('can_interface')}
        ]
    )

    return LaunchDescription([
        can_interface_arg,
        act_controller_node
    ])
