import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare the launch options

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')

    color_arg = DeclareLaunchArgument(
        'color',
        default_value='yellow',
        description='Color of the robot (yellow|blue)',
    )

    # =========================== NODES NEEDED BOTH IN SIMULATION AND ON REAL ROBOT ===========================

    sm = Node(
        package='champi_brain',
        executable='state_machine_itf.py',
        name='sm_ros_itf',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        color_arg,
        sm
    ])