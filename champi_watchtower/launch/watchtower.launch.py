from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('champi_vision').find('champi_vision')
    
    # Declare launch arguments
    camera_info_arg = DeclareLaunchArgument(
        'camera_info_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'calib', 'simu_cam.yaml']),
        description='Path to camera calibration file'
    )
    
    table_ref_image_arg = DeclareLaunchArgument(
        'table_reference_image',
        default_value=PathJoinSubstitution([pkg_share, '..', 'champi_vision', 'ressources', 'images', 'table.png']),
        description='Path to table reference image for calibration'
    )
    
    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.07',
        description='Size of ArUco markers in meters'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Rate for publishing robot poses (Hz)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/watchtower/camera/image_color',
        description='Topic name for camera images'
    )
    
    is_simu_arg = DeclareLaunchArgument(
        'is_simu_with_webots',
        default_value='true',
        description='Whether running in Webots simulation'
    )
    
    marker_id_min_arg = DeclareLaunchArgument(
        'marker_id_min',
        default_value='0',
        description='Minimum ArUco marker ID to detect'
    )
    
    marker_id_max_arg = DeclareLaunchArgument(
        'marker_id_max',
        default_value='10',
        description='Maximum ArUco marker ID to detect'
    )
    
    # Watchtower node
    watchtower_node = Node(
        package='champi_watchtower',
        executable='watchtower_node.py',
        name='watchtower_node',
        output='screen',
        parameters=[{
            'camera_info_file': LaunchConfiguration('camera_info_file'),
            'table_reference_image': LaunchConfiguration('table_reference_image'),
            'marker_length': LaunchConfiguration('marker_length'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': '/watchtower/camera_info',
            'is_simu_with_webots': LaunchConfiguration('is_simu_with_webots'),
            'marker_id_min': LaunchConfiguration('marker_id_min'),
            'marker_id_max': LaunchConfiguration('marker_id_max'),
        }]
    )
    
    return LaunchDescription([
        camera_info_arg,
        table_ref_image_arg,
        marker_length_arg,
        publish_rate_arg,
        image_topic_arg,
        is_simu_arg,
        marker_id_min_arg,
        marker_id_max_arg,
        watchtower_node,
    ])
