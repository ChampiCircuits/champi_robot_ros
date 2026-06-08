#!/usr/bin/env python

import os
import launch
import launch.actions
import launch.event_handlers
import launch.events
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController



def generate_launch_description():
    mode = "realtime"

    # TODO It would be cleaner to copy files in share/ ...
    world_path = os.path.expanduser("~/champi_ws/src/champi_robot_ros/champi_webots/worlds/champi_webots.wbt")

    webots = WebotsLauncher(
        world=world_path,
        mode=mode,
        ros2_supervisor=False
    )

    # This action will kill all nodes once the Webots simulation has exited
    kill_nodes = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )

    # Webots ROS2 controller to publish camera data
    robot_description_path = os.path.expanduser("~/champi_ws/src/champi_robot_ros/champi_webots/urdf/watchtower.urdf")
    watchtower_controller = WebotsController(
        robot_name='watchtower',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': False,
             'set_robot_state_publisher': False,
             'publish_tf': False},
        ],
        respawn=True
    )

    return LaunchDescription([
        webots,
        #webots._supervisor, # Provides additional topics to interact with webots ; not strictly needed, let's uncomment only if we need because I'm scared it's gonna bring problems
        kill_nodes,
        watchtower_controller,
    ])