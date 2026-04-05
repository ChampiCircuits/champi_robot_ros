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

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='in simulation (True|False)',
    )

    # =========================== NODES NEEDED BOTH IN SIMULATION AND ON REAL ROBOT ===========================
    sim_config = LaunchConfiguration('sim')

    sm_node = Node(
        package='champi_brain',
        executable='state_machine_node.py',
        name='state_machine_node',
        namespace='champi_brain',
        output='screen',
        respawn=True,
        parameters=[config_file_path, {'sim': sim_config}],
    )

    world_state_node = Node(
        package='champi_brain',
        executable='world_state_node.py',
        name='world_state_node',
        namespace='champi_brain',
        output='screen',
        respawn=True,
        parameters=[config_file_path, {'sim': sim_config}],
    )

    return LaunchDescription([
        sim_arg,
        sm_node,
        world_state_node
    ])