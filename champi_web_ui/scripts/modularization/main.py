#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
import threading
from pages import launch_page, match_page, debug_page, diagnostics_page, ip_page, usages_page, in_match_page, strategies_page, speeds_page

from nicegui import ui
from node import init_ros_node

ros_node = init_ros_node()

def ros_spin():
    try:
        rclpy.spin(ros_node)
    except ExternalShutdownException:
        ros_node.destroy_node()
        rclpy.shutdown()


threading.Thread(target=ros_spin, daemon=True).start()

launch_page.create()
match_page.create()
strategies_page.create()
in_match_page.create()
debug_page.create()
diagnostics_page.create()
ip_page.create()
usages_page.create()
speeds_page.create()


ui.run(title='Champi Web UI')