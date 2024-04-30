import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    camera_node = Node(
            package='champi_camera_yolo',
            executable='camera_node.py',
            name='champi_camera_yolo',
            output='screen',
    )

    return LaunchDescription([
        camera_node,
    ])