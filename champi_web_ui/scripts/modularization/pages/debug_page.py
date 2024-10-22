import theme

from nicegui import ui, events
from std_msgs.msg import Int64, Int64MultiArray, Empty, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from enum import Enum
from math import acos, sin, cos, pi

from node import init_ros_node
from utils import real_to_px, px_to_real, id_to_coords


def pose_from_position(position, stamp): # TODO, à importer de utils dans champi_brain
    goal_pose_msg = PoseStamped()
    goal_pose_msg.header.frame_id = 'map'
    goal_pose_msg.header.stamp = stamp
    goal_pose_msg.pose.position.x = position[0]
    goal_pose_msg.pose.position.y = position[1]
    goal_pose_msg.pose.position.z = 0.0
    # theta radians to quaternion
    goal_pose_msg.pose.orientation.x = 0.0
    goal_pose_msg.pose.orientation.y = 0.0
    goal_pose_msg.pose.orientation.z = sin(position[2] / 2)
    goal_pose_msg.pose.orientation.w = cos(position[2] / 2)
    return goal_pose_msg

class CAN_MSGS(Enum): #TODO jsp comment l'importer de champi_brain.utils
    START_GRAB_PLANTS = 0
    STOP_GRAB_PLANTS = 1
    RELEASE_PLANT = 2
    TURN_SOLAR_PANEL = 3
    INITIALIZING = 4
    FREE = 5

toggle_pose_effect_value = False
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

MOVE_ROBOT_STRING = "Déplacer le robot"
INIT_ROBOT_POSE_STRING = "Choisir la position de départ"

robot_rotation_normalized_to_one = 0
robot_pose = None

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

    x,y = id_to_coords(id)

    interactive_image_table.content = svg_zones_overlay + svg_plants_overlay + '''<circle id="P3" cx="{x}" cy="{y}" r="470" fill="black" stroke="black" />'''.format(x=x,y=y)

    print(px_to_real((x, y, 0)))
    print(toggle_pose_effect_value)
    if toggle_pose_effect_value == INIT_ROBOT_POSE_STRING:
        msg = String()
        msg.data = id
        zone_pub.publish(msg)
    elif toggle_pose_effect_value == MOVE_ROBOT_STRING:
        goal_pose_publisher.publish(pose_from_position(px_to_real((x, y, 0)), ros_node.get_clock().now().to_msg()))


def update_robot_position(odom_msg: Odometry):
    if interactive_image_table is None:
        return
    
    x,y,z,w = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
    theta = -2*acos(w)+1.57

    global robot_pose
    robot_pose = (x,y,theta)

    global robot_rotation_normalized_to_one
    robot_rotation_normalized_to_one = abs(theta % 2*pi)/(2*pi)
    # print(robot_rotation_normalized_to_one, theta)

    x_px, y_px, theta = real_to_px((x, y, theta))

    interactive_image_table.content = svg_zones_overlay + '''<circle id="robot" cx="{x_px}" cy="{y_px}" r="150" fill="red" stroke="red" />'''.format(x_px=x_px,y_px=y_px)
    if toggle_pose_effect_value == MOVE_ROBOT_STRING:
        interactive_image_table.content += svg_plants_overlay
    interactive_image_table.update()

def joystick_update(event):
    x, y = event.x, event.y # ]-1,1[ both
    print(x,y)

    msg = Twist()
    msg.linear.x = x//10
    msg.linear.y = -y//10
    msg.angular.z = 0.
    cmd_vel_publisher.publish(msg)

def knob_update(event):
    theta = event.value * 2*pi - pi

    x, y = robot_pose[0], robot_pose[1]
    print(x,y,theta)

    msg = PoseStamped()
    msg.header.stamp = ros_node.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = sin(theta/2)
    msg.pose.orientation.w = cos(theta/2)

    goal_pose_publisher.publish(msg)

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
                                ui.label().classes('text-h4 text-black-8').bind_text_from(globals(), 'CAN_state')

                        ToggleButtonGrabPlants('Start Grabbing Plants...')

                    with ui.card().style('align-items: center'):
                        with ui.row():
                            ui.label('Plants in the robot:').classes('text-h4 text-grey-8')
                            ui.label().classes('text-h4 text-black-8').bind_text_from(globals(), 'nb_plants')
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
                        ui.toggle([MOVE_ROBOT_STRING, INIT_ROBOT_POSE_STRING], value=MOVE_ROBOT_STRING).bind_value_to(globals(), 'toggle_pose_effect_value')
                        global interactive_image_table
                        interactive_image_table = ui.interactive_image(src, content=svg_zones_overlay+svg_plants_overlay).on('svg:pointerdown', zone_chosen).style('width:100%')

                    with ui.card().style('align-items: center'):
                        def tirette_publish():
                            e = Empty()
                            tirette_pub.publish(e)
                        ui.button('Tirette', on_click=tirette_publish)

                        with ui.row():
                            with ui.column():
                                ui.joystick(color='green', size=100, on_move=joystick_update).style('width:95%')
                                ui.label('joystick').classes('text-h6 text-grey-8')

                            with ui.knob(color='orange', track_color='blue', size='9vmax', on_change=knob_update).bind_value(globals(), robot_rotation_normalized_to_one):
                                ui.icon('3d_rotation')
                
                
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

    if CAN_state == CAN_MSGS.START_GRAB_PLANTS:
        CAN_state = "START_GRAB_PLANTS"
    elif CAN_state == CAN_MSGS.STOP_GRAB_PLANTS:
        CAN_state = "STOP_GRAB_PLANTS"
    elif CAN_state == CAN_MSGS.RELEASE_PLANT:
        CAN_state = "RELEASE_PLANT"
    elif CAN_state == CAN_MSGS.TURN_SOLAR_PANEL:
        CAN_state = "TURN_SOLAR_PANEL"
    elif CAN_state == CAN_MSGS.INITIALIZING:
        CAN_state = "INITIALIZING"
    elif CAN_state == CAN_MSGS.FREE:
        CAN_state = "FREE"

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
tirette_pub = ros_node.create_publisher(Empty, '/tirette_start', 10)
odom_subscriber = ros_node.create_subscription(Odometry, '/odometry/filtered', update_robot_position, 10)
goal_pose_publisher = ros_node.create_publisher(PoseStamped, '/goal_pose', 10)
zone_pub = ros_node.create_publisher(String, '/start_zone', 10)
cmd_vel_publisher = ros_node.create_publisher(Twist,'/cmd_vel_stop',10)


# TODO update CAN state et nb plants