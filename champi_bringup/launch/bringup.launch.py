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

    imu_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_controllers'),
            '/launch/imu_controller.launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    act_controller_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_controllers'),
            '/launch/act_controller.launch.py'
        ]),
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

  # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            {'range_min': 0.05},
            {'range_max': 5.0},
            # {'units': 'M'},
        ],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    lidar_simu_node = Node(
        package='champi_simulator',
        executable='simu_lidar_node.py',
        name='simu_lidar_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim'))
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

    # cmd_vel multiplexer
    cmd_vel_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen', # TODO tester output='both'
        parameters=[config_file_path],
        remappings=[('/cmd_vel_out', '/base_controller/cmd_vel')]
    )

    # Teleop
    teleop_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_bringup'),
            '/launch/teleop.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('joy'))
    )

    pub_goal_rviz_node = Node(
        package='champi_nav2',
        executable='pub_goal_rviz.py',
        name='pub_goal_rviz',
        output='screen'
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

    laser_filter = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[config_file_path]
    )


    raspi_cam_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('champi_vision'),
            '/launch/raspi_cam.launch.py'
        ])
    )


    visual_loc = Node(
            package="champi_vision",
            executable="visual_loc_node.py",
            name='visual_loc',
            output='screen'
    )


    delayed_7s = TimerAction(period=7., actions=[ldlidar_node, laser_filter])
    delayed_5s = TimerAction(period=5., actions=[ukf_node])
    delayed_4s = TimerAction(period=4., actions=[visual_loc])
    delayed_3s = TimerAction(period=3., actions=[raspi_cam_launch])
    delayed_2s = TimerAction(period=2., actions=[imu_controller_launch])
    delayed_1s = TimerAction(period=1., actions=[act_controller_launch])
    

    return LaunchDescription([
        sim_arg,
        joy_arg,
        static_tf_map_odom,
        description_broadcaster,
        base_controller_launch,
        lidar_simu_node,
        base_control_simu_node,
        cmd_vel_mux_node,
        teleop_launch,
        pub_goal_rviz_node,
        delayed_1s,
        delayed_2s,
        delayed_3s,
        delayed_4s,
        delayed_5s,
        delayed_7s
    ])

