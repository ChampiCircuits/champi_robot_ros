#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
import threading
from pages import launch_page, match_page, debug_page, diagnostics_page, ip_page, usages_page, in_match_page, strategies_page

from nicegui import ui
from node import init_ros_node

print("web ui main launched")

node = init_ros_node()

def ros_spin():
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


threading.Thread(target=ros_spin, daemon=True).start()

launch_page.create()
match_page.create()
strategies_page.create()
in_match_page.create()
debug_page.create()
diagnostics_page.create()
ip_page.create()
usages_page.create()

# project_dir = os.path.dirname(os.path.abspath(__file__))
# print("✅ Limiting reload to:", project_dir)

print('pages created')
print('launching ui')

ui.run(
    title='Champi Web UI',
    show=False,
    reload=False,
    binding_refresh_interval=0.5
    # uvicorn_reload_dirs='/home/champi/champi_ws/src/champi_robot_ros/champi_web_ui/scripts/modularization/',
    # uvicorn_reload_includes='*.py',
    # uvicorn_reload_excludes='.*,__pycache__,*.pyc,*.pyo,*.swp,*.tmp,lost+found,*sys/*,*dev/*,*proc/*,*tmp/*',
)
