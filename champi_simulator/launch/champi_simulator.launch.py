import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xml.etree.ElementTree as ET


def replace_absolute_paths(xml_file_path):
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Define the prefix for relative paths
    relative_prefix = "model://"

    # Find all elements with 'uri' attribute
    for uri_elem in root.iter('uri'):
        absolute_path = uri_elem.text.strip()
        if absolute_path.startswith("file:///"):
            # Convert absolute path to relative path
            relative_path = relative_prefix + os.path.basename(absolute_path)
            # Update the text content with the relative path
            uri_elem.text = relative_path

    # Save the modified XML file
    tree.write(xml_file_path)


def generate_launch_description():

    # Get the path to the package
    champi_simulator_dir = get_package_share_directory('champi_simulator')

    # Replace absolute paths in the world file. Idk why gazebo uses absolute ones :(
    replace_absolute_paths(os.path.join(champi_simulator_dir, 'worlds', 'cup_world.sdf'))

    # Declare the launch arguments

    world_file = LaunchConfiguration('world_file')
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(champi_simulator_dir, 'worlds', 'cup_world_objects.sdf'),
        description='Path to the world file'
    )

    # Set Gazebo resource path
    gz_sim_resource_path = os.path.join(champi_simulator_dir, 'models')
    print("ressource path set to: ", gz_sim_resource_path)
    # Idk why, but the environment variable has to be set in ld.add_action, otherwise it doesn't work
    # set_ressource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_sim_resource_path),

    # Launch the Gazebo Harmonic simulator with the specified world file
    # ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=
    launch_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [world_file, ' -v 1']}.items(),
    )

    # Config file for the bridge... TODO doesn't work for now.
    # See https://github.com/gazebosim/ros_gz/issues/354
    # And https://stackoverflow.com/questions/74857677/top-level-must-be-a-yaml-sequence-error-in-ros2-even-with-yaml-being-a-sequence
    bridge_config = os.path.join(champi_simulator_dir, 'config', 'ros_gz_bridge.yaml')

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
    # ld.add_action(set_ressource_path)
    ld.add_action(SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_sim_resource_path))
    ld.add_action(declare_world_file_cmd)
    ld.add_action(launch_gz_sim)
    ld.add_action(launch_bridge_node)

    return ld
