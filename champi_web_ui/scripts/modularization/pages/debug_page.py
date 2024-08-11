import theme
from message import message

from nicegui import ui, app, ui_run
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
from enum import Enum
import threading
from pathlib import Path
from rclpy.executors import ExternalShutdownException

from node import init_ros_node

class CAN_MSGS(Enum): #TODO jsp comment l'importer de champi_brain.utils
    START_GRAB_PLANTS = 0
    STOP_GRAB_PLANTS = 1
    RELEASE_PLANT = 2
    TURN_SOLAR_PANEL = 3
    INITIALIZING = 4
    FREE = 5

#################################################
#################### PAGE #######################
#################################################
class ToggleButtonGrabPlants(ui.button):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._state = False
        self.on('click', self.toggle)
        self.props(f'color=green')

    def toggle(self) -> None:
        """Toggle the button state."""
        self._state = not self._state
        if self._state:
            self.props(f'color=red')
            self.set_text("Stop Grabbing Plants")
            start_grab_plants()
        else:
            self.props(f'color=green')
            self.set_text("Start Grabbing Plants")
            stop_grab_plants()
        self.update()

    def update(self) -> None:
        super().update()

@ui.refreshable
def create() -> None:
    @ui.page('/debug')
    def page_a():
        with theme.frame('Debug'):

            with ui.row():
                with ui.card():
                    message('CAN State:')
                    with ui.element('div').classes('p-3 bg-green-100'):
                        label_CAN_state = ui.label(CAN_state)
                    ui.timer(1.0, lambda: label_CAN_state.set_text(CAN_state))


                    message('Commands')
                    ToggleButtonGrabPlants('Start Grabbing Plants...')

                with ui.card():
                    message('Plants in the robot:')
                    with ui.element('div').classes('p-3 bg-green-100'):
                        label_CAN_state = ui.label(nb_plants)
                    ui.timer(1.0, lambda: label_CAN_state.set_text(nb_plants))

                    ui.button('Release 1 plant',on_click= release_plant)


#################################################
#################### UTILS ######################
#################################################
def act_sub_update(msg):
    global CAN_state, nb_plants
    CAN_state = msg.data[0]
    nb_plants = msg.data[1]
        # START_GRAB_PLANTS = 0
        # STOP_GRAB_PLANTS = 1
        # RELEASE_PLANT = 2
        # TURN_SOLAR_PANEL = 3
        # INITIALIZING = 4
        # FREE = 5

    if CAN_state == 0:
        CAN_state = "START_GRAB_PLANTS"
    elif CAN_state == 1:
        CAN_state = "STOP_GRAB_PLANTS"
    elif CAN_state == 2:
        CAN_state = "RELEASE_PLANT"
    elif CAN_state == 3:
        CAN_state = "TURN_SOLAR_PANEL"
    elif CAN_state == 4:
        CAN_state = "INITIALIZING"
    elif CAN_state == 5:
        CAN_state = "FREE"
    # CAN_state_label.config(text=CAN_state) # TODO

def start_grab_plants():
    print("start_grab_plants")
    publish_on_CAN(CAN_MSGS.START_GRAB_PLANTS) # TODO

def stop_grab_plants():
    print("stop_grab_plants")
    publish_on_CAN(CAN_MSGS.STOP_GRAB_PLANTS)

def release_plant():
    print("release_plant")
    publish_on_CAN(CAN_MSGS.RELEASE_PLANT)

def publish_on_CAN( message):
    # publish on the topic "/CAN" to be forwarded by CAN by the API
    msg = Int64()
    if message == CAN_MSGS.START_GRAB_PLANTS:
        msg.data = 0
    elif message == CAN_MSGS.STOP_GRAB_PLANTS:
        msg.data = 1
    elif message == CAN_MSGS.RELEASE_PLANT:
        msg.data = 2
    elif message == CAN_MSGS.TURN_SOLAR_PANEL:
        msg.data = 3
    ros_node.CAN_pub.publish(msg)
    

CAN_state = None
nb_plants = 0

ros_node = init_ros_node()
ros_node.create_subscription(Int64MultiArray, '/act_status', act_sub_update, 10)
