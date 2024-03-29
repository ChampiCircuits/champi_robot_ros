from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    calib_file = 'file://' + os.path.join(get_package_share_directory('champi_vision'), 'config', 'calib', 'raspi_cam_robotik.yaml')

    print(calib_file)

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[
                {"image_size": [1640, 1232]},
                # {"time_per_frame": [1, 5]},
                {"camera_info_url": calib_file}
        ]
        )
    ])
# TODO meilleure perf CPU avec gpu memoy + faible ! car cela limitait artificiellement les FPS