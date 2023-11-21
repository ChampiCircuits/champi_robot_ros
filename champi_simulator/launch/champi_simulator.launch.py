import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the path to the package
    champi_simulator_dir = get_package_share_directory('champi_simulator')

    # Declare the launch arguments
    world_file = LaunchConfiguration('world_file')
    gz_sim_resource_path = LaunchConfiguration('gz_sim_resource_path')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(champi_simulator_dir, 'worlds', 'cup_world.sdf'),
        description='Path to the world file'
    )

    declare_gz_sim_resource_path_cmd = DeclareLaunchArgument(
        'gz_sim_resource_path',
        default_value=os.path.join(champi_simulator_dir, 'models'),
        description='Path to the Gazebo simulator resource directory'
    )

    # Config file for the bridge
    bridge_config = os.path.join(champi_simulator_dir, 'config', 'ros_gz_bridge.yaml')

    # Set the environment variable GZ_SIM_RESOURCE_PATH
    # set_gz_sim_resource_path_cmd = ExecuteProcess(
    #     cmd=['env', f'GZ_SIM_RESOURCE_PATH={gz_sim_resource_path}'],
    #     output='screen'
    # )

    # Launch the Gazebo Harmonic simulator with the specified world file
    # ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=
    launch_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [world_file, ' -v 1']}.items()
    )

    # Launch the Gazebo ROS bridge for camera and camera_info topics
    launch_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/champi/sensors/camera_1@sensor_msgs/msg/Image@gz.msgs.Image',
            '/champi/sensors/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
    )
    # Create the launch description and add the commands
    ld = LaunchDescription()
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_gz_sim_resource_path_cmd)
    # ld.add_action(set_gz_sim_resource_path_cmd)
    ld.add_action(launch_gz_sim)
    ld.add_action(launch_bridge_node)

    return ld
