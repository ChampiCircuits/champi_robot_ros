import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    color_arg = DeclareLaunchArgument(
        'color',
        description='Color of the robot (yellow|blue)',
    )
    
    screen_manager = Node(
            package='champi_brain',
            executable='screen_manager.py',
            name='champi_brain',
            output='screen',
    )

    rviz_markers = Node(
            package='champi_brain',
            executable='rviz_markers.py',
            name='rviz_markers',
            output='screen',
    )

    strat = Node(
            package='champi_brain',
            executable='strategy_engine_node.py',
            name='strategy',
            output='screen',
            parameters=[{'color': LaunchConfiguration('color')}]
    )

    return LaunchDescription([
        color_arg,
        # screen_manager,
        # rviz_markers,
        strat
    ])