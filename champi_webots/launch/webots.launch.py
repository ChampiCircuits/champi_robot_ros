#!/usr/bin/env python

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.urdf_spawner import URDFSpawner
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('champi_webots')
    mode = "realtime"

    # TODO It would be cleaner to copy files in share/ ...
    world_path = os.path.expanduser("~/champi_ws/src/champi_robot_ros/champi_webots/worlds/champi_webots.wbt")

    webots = WebotsLauncher(
        world=world_path,
        mode=mode,
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'urdf', 'watchtower.urdf')
    controller = WebotsController(
        robot_name='watchtower_1',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': False,
             'set_robot_state_publisher': True},
        ],
        respawn=True
    )


    # spawn_URDF_robot = URDFSpawner(
    #     name='Table',
    #     urdf_path=robot_description_path,
    #     translation='0 0 1',
    #     rotation='0 0 1 -1.5708',
    # )


    return LaunchDescription([
        # webots,
        webots._supervisor,
        controller,
        # spawn_URDF_robot,
    ])