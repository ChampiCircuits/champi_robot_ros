import theme

from nicegui import ui, events
from std_msgs.msg import Empty, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
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
    goal_pose_msg.pose.orientation.z = 0.0
    goal_pose_msg.pose.orientation.z = 0.0
    goal_pose_msg.pose.orientation.z = sin(position[2] / 2)
    goal_pose_msg.pose.orientation.w = cos(position[2] / 2)
    return goal_pose_msg

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

def create() -> None:
    @ui.page('/debug')
    def page_a():
        with theme.frame('Debug'):
            with ui.grid(columns=1).style('width: 100%'):
                with ui.element('div'):
                    with ui.card():
                        ui.label('Contrôle').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('RESET [SELECT]', on_click=lambda: ros_node.send_actuator_action('RESET_ACTUATORS')).props('color=orange')
                            def tirette_publish():
                                e = Empty()
                                tirette_pub.publish(e)
                            ui.button('Tirette', on_click=tirette_publish)

                        ui.label('Thermomètre').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('Relever [L1]', on_click=lambda: ros_node.send_actuator_action('THERMOMETER_RAISE_SERVO')).props('color=blue')
                            ui.button('Abaisser [L2]', on_click=lambda: ros_node.send_actuator_action('THERMOMETER_LOWER_SERVO')).props('color=blue')

                        ui.label('Ascenseur + Pince').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('Prendre 2 boîtes [A]', on_click=lambda: ros_node.send_actuator_action('TAKE_2_BOXES')).props('color=green')
                            ui.button('Monter 2 boîtes [A↑]', on_click=lambda: ros_node.send_actuator_action('BRING_2_BOXES_ON_TOP')).props('color=green')
                            ui.button('Poser les 2 dernières boîtes devant [A↓]', on_click=lambda: ros_node.send_actuator_action('PUT_2_LAST_BOXES_ON_THE_GROUND')).props('color=green')

                        ui.label('Tri + pushers').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('Préparer le pusher [Y]', on_click=lambda: ros_node.send_actuator_action('PREPARE_TOP_PUSHER')).props('color=purple')
                            ui.button('Prendre & Trier 2 boîtes de l ascenseur [Y←]', on_click=lambda: ros_node.send_actuator_action('GRAB_AND_SORT_2_BOXES_FROM_LIFT')).props('color=purple')
                            ui.button('Sortir 2 boîtes[X]', on_click=lambda: ros_node.send_actuator_action('PUSH_2_BOXES_OUT')).props('color=purple')

                        ui.label('Rampe').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('Ouvrir [B]', on_click=lambda: ros_node.send_actuator_action('OPEN_EXIT_RAMP')).props('color=orange')

                        ui.label('Debug').classes('text-subtitle2 text-grey-6 q-mt-sm')
                        with ui.row().classes('q-gutter-sm'):
                            ui.button('GET READY', on_click=lambda: ros_node.send_actuator_action('GET_READY')).props('color=green')
                            ui.button('STOP MOTEURS', on_click=lambda: ros_node.send_actuator_action('STOP_ALL_MOTORS')).props('color=red')
                            ui.button('ACTIVER MOTEURS', on_click=lambda: ros_node.send_actuator_action('ENABLE_ALL_MOTORS')).props('color=blue')


#################################################
#################### UTILS ######################
#################################################

ros_node = init_ros_node()
tirette_pub = ros_node.create_publisher(Empty, '/tirette_start', 10)
odom_subscriber = ros_node.create_subscription(Odometry, '/odometry/filtered', update_robot_position, 10)
goal_pose_publisher = ros_node.create_publisher(PoseStamped, '/goal_pose', 10)
zone_pub = ros_node.create_publisher(String, '/start_zone', 10)
cmd_vel_publisher = ros_node.create_publisher(Twist,'/cmd_vel_stop',10)