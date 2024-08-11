import theme

from nicegui import ui, events
from std_msgs.msg import Int64, Int64MultiArray
from enum import Enum

from node import init_ros_node

class CAN_MSGS(Enum): #TODO jsp comment l'importer de champi_brain.utils
    START_GRAB_PLANTS = 0
    STOP_GRAB_PLANTS = 1
    RELEASE_PLANT = 2
    TURN_SOLAR_PANEL = 3
    INITIALIZING = 4
    FREE = 5

interactive_image_table = None

src = 'champi_web_ui/scripts/modularization/resources/table_2024.png'

svg_zones_overlay = '''
                <rect id="B1" x="0" y="0" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                <rect id="Y1" x="0" y="2925" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
                <rect id="B2" x="0" y="5855" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                
                <rect id="Y2" x="9640" y="0" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
                <rect id="B3" x="9640" y="2925" width="1700" height="1700" stroke="#4E84A2" fill="#4E84A2" pointer-events="all" cursor="pointer" />
                <rect id="Y3" x="9640" y="5855" width="1700" height="1700" stroke="#E2BB4E" fill="#E2BB4E" pointer-events="all" cursor="pointer" />
            '''
svg_plants_overlay = '''
                <circle id="P1" cx="3785" cy="2645" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P2" cx="5675" cy="1895" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P3" cx="7560" cy="2645" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />

                <circle id="P4" cx="3785" cy="4910" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P5" cx="5675" cy="5660" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
                <circle id="P6" cx="7560" cy="4910" r="470" fill="green" stroke="green" pointer-events="all" cursor="pointer" />
            '''

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

def zone_chosen(args: events.GenericEventArguments):
    id = args.args['element_id']
    print(id)
    match id:
        case "B1":
            x=1700//2
            y=1700//2
        case "B2":
            x=850
            y=5855+1700//2
        case "B3":
            x=9640+1700//2
            y=2925+1700//2
        case "Y1":
            x=1700//2
            y=2925+1700//2
        case "Y2":
            x=9640+1700//2
            y=1700//2
        case "Y3":
            x=9640+1700//2
            y=5855+1700//2

    interactive_image_table.content = svg_zones_overlay + svg_plants_overlay + '''<circle id="P3" cx="{x}" cy="{y}" r="470" fill="black" stroke="black" />'''.format(x=x,y=y)

@ui.refreshable
def create() -> None:
    @ui.page('/debug')
    def page_a():
        with theme.frame('Debug'):
            with ui.grid(columns=3).style('width: 100%'):
                with ui.element('div'):
                    with ui.card().style('align-items: center'):
                        with ui.row():
                            ui.label('START_GRAB_PLANTS = 0; STOP_GRAB_PLANTS = 1; RELEASE_PLANT = 2; TURN_SOLAR_PANEL = 3; INITIALIZING = 4; FREE = 5')
                            ui.label('CAN State:').classes('text-h4 text-grey-8')
                            with ui.element('div'):
                                label_CAN_state = ui.label(CAN_state).classes('text-h4 text-black-8')

                        ToggleButtonGrabPlants('Start Grabbing Plants...')

                    with ui.card().style('align-items: center'):
                        with ui.row():
                            ui.label('Plants in the robot:').classes('text-h4 text-grey-8')
                            label_CAN_state = ui.label(nb_plants).classes('text-h4 text-black-8')
                        ui.button('Release 1 plant',on_click= release_plant).props('color=pink')

                    with ui.card().style('align-items: center'):
                        with ui.row():
                            with ui.column():
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                            with ui.column():
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                            with ui.column():
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)
                                ui.button('xxxxxxxxxxxxxx', on_click=release_plant)

                with ui.column():
                    with ui.card().style('align-items: center'):
                        ui.label("TODO : choix zone et tirette simu")
                        global interactive_image_table
                        interactive_image_table = ui.interactive_image(src, content=svg_zones_overlay+svg_plants_overlay).on('svg:pointerdown', zone_chosen).style('width:100%')

                        ui.button('Tirette')
                    with ui.card().style('align-items: center'):
                        "..."
                
                
                with ui.card().style('align-items: center'):
                    "..."


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
    

CAN_state = "?"
nb_plants = 0

ros_node = init_ros_node()
ros_node.create_subscription(Int64MultiArray, '/act_status', act_sub_update, 10)


# TODO update CAN state et nb plants