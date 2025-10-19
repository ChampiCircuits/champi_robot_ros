#!/usr/bin/env python

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('champi_webots')
    mode = LaunchConfiguration('mode')

    # TODO It would be cleaner to copy files in share/ ...
    world_path = os.path.expanduser("~/champi_ws/src/champi_robot_ros/champi_webots/worlds/champi_webots.wbt")

    webots = WebotsLauncher(
        world=world_path,
        mode=mode,
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'urdf', 'table.urdf')
    controller = WebotsController(
        robot_name='Table',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': False,
             'set_robot_state_publisher': True},
        ],
        respawn=True
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_burger_example.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        controller,
    ])