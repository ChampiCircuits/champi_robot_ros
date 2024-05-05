from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

import os

def generate_launch_description():

    # ros2 run tf2_ros static_transform_publisher 0 0 0.395 -1.509 3.14159 -1 base_link camera

    return LaunchDescription([

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0.78539', '-1.57079', '3.14159', '-1', 'base_link', 'camera']
        # ),
        Node(
            package='champi_vision',
            executable='visual_loc_node.py',
            name='visual_loc_node',
            output='screen'
        ),
        # Node(
        #     package='dev_tools',
        #     executable='camera_info_publisher_node.py',
        #     name='camera_info_publisher',
        #     output='screen',
        #     parameters=[{
        #         "calib_yaml_path": os.path.join(get_package_share_directory('champi_vision'), 'config', 'calib', 'raspi_cam_robotik.yaml')
        #     }]
        # )
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', '/home/andre/rosbag2_2024_05_03-20_40_15'],
        #     output='screen'
        # )

    ])