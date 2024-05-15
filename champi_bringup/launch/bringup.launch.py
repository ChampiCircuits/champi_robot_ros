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

    # Declare the launch options
    sim_arg = DeclareLaunchArgument(
        'sim',
        description='Launch simulation (true|false)',
    )

    base_arg = DeclareLaunchArgument(
        'base',
        default_value='True',
        description='Launch base (true|false)',
    )

    nav_arg = DeclareLaunchArgument(
        'nav',
        default_value='False',
        description='Launch navigation (true|false)',
    )

    camera_perception_arg = DeclareLaunchArgument(
        'cam',
        default_value='False',
        description='Launch camera perception (true|false)',
    )

    lidar_perception_arg = DeclareLaunchArgument(
        'lidar',
        default_value='False',
        description='Launch lidar perception (true|false)',
    )

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='False',
        description='Launch joystick (true|false)',
    )

    act_arg = DeclareLaunchArgument(
        'act',
        default_value='False',
        description='Launch actuator controller (true|false)',
    )

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')


    base_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/base.launch.py'
        ]),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items(),
        condition=IfCondition(LaunchConfiguration('base'))
    )

    nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/nav.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    camera_perception_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/camera_perception.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('cam'))
    )

    lidar_perception_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/lidar_perception.launch.py'
        ]),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items(),
        condition=IfCondition(LaunchConfiguration('lidar'))
    )

    teleop_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/teleop.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('joy'))
    )

    act_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_controllers'),
            '/launch/act_controller.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('act'))
    )



    return LaunchDescription([
        sim_arg,
        base_arg,
        nav_arg,
        camera_perception_arg,
        lidar_perception_arg,
        joy_arg,
        act_arg,

        base_launch,
        nav_launch,
        camera_perception_launch,
        lidar_perception_launch,
        teleop_launch,
        act_controller_launch
    ])

