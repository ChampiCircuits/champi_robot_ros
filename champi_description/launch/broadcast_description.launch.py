import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    urdf_file_path = os.path.join(get_package_share_directory('champi_description'), 'urdf', 'champi.urdf')
    urdf_content = open(urdf_file_path).read()

    description_broadcaster = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
    )


    return LaunchDescription([
        description_broadcaster
    ])

