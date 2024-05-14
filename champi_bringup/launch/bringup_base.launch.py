import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


"""
Ce launch file lance les nodes liés à la base et à la localisation du robot.
TODO: il faudrait l'appeler depuis le bringup principal, pour ne pas répéter le code.
"""

def generate_launch_description():

    # Declare the launch options
    sim_arg = DeclareLaunchArgument(
        'sim',
        description='Launch simulation (true|false)',
    )

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='False',
        description='Launch joystick (true|false)',
    )

    # Get configuration file
    config_file_path = os.path.join(get_package_share_directory('champi_bringup'), 'config', 'champi.config.yaml')


    # Get the URDF file TODO faire ça dans un launch file dans champi_description plutôt
    urdf_file_path = os.path.join(get_package_share_directory('champi_description'), 'urdf', 'champi.urdf')
    urdf_content = open(urdf_file_path).read()

    description_broadcaster = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    base_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_controllers'),
            '/launch/base_controller.launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    base_control_simu_node = Node(
        package='champi_simulator',
        executable='holo_base_control_simu_node.py',
        name='base_controller_simu',
        output='screen',
        parameters=[config_file_path],
        remappings=[('/cmd_vel', '/base_controller/cmd_vel')],
        condition=IfCondition(LaunchConfiguration('sim'))
    )


    imu_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_controllers'),
            '/launch/imu_controller.launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("champi_bringup"), "config", "ukf.yaml")],
        remappings=[('/cmd_vel', '/base_controller/cmd_vel_limited')]
    )

    # Static transform map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    # cmd_vel multiplexer
    cmd_vel_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen', # TODO tester output='both'
        parameters=[config_file_path],
        remappings=[('/cmd_vel_out', '/base_controller/cmd_vel')]
    )

    return LaunchDescription([
        sim_arg,
        joy_arg,
        static_tf_map_odom,
        description_broadcaster,
        base_controller_launch,
        base_control_simu_node,
        cmd_vel_mux_node,
        imu_controller_launch,
        ukf_node
    ])

