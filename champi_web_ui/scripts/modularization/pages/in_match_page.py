import theme

from nicegui import ui, app

from node import init_ros_node
import time

ros_node = init_ros_node()

score_label = None
time_left_label = None
sm_state_label = None
sm_state_desc_label = None
odom_label = None
speed_label = None
distance_label = None



#################################################
#################### PAGE #######################
#################################################

def create() -> None:
    @ui.page('/in_match')
    def page_in_match():

        with theme.frame('In Match Page'):
            with ui.column().style("align-items: center"):
                global score_label, time_left_label # They will stay to None, until the page is loaded (so ros2 callback can't access it)
                ui.label('Score :').classes('text-h4 text-grey-8')
                score_label = ui.label(str(ros_node.score) + ' points').classes('text-h4 text-black-8')

                ui.separator()

                ui.label('Temps restant :').classes('text-h4 text-grey-8')
                time_left_label = ui.label(str(ros_node.time_left) + ' secondes').classes('text-h4 text-black-8')

        # Place this overlay outside theme.frame (which uses absolute-center),
        # otherwise fixed positioning is constrained to the centered container.
        global sm_state_label, sm_state_desc_label, odom_label, speed_label, distance_label
        with ui.column().style('position: fixed; bottom: 0; right: 0; z-index: 9999; align-items: flex-end; gap: 2px; padding: 8px;'):
            sm_state_label = ui.label('').classes('text-h6 text-grey-10')
            sm_state_desc_label = ui.label('').classes('text-subtitle2 text-grey-7')
            odom_label = ui.label('').classes('text-subtitle2 text-grey-8')
            speed_label = ui.label('').classes('text-subtitle2 text-grey-8')
            distance_label = ui.label('').classes('text-subtitle2 text-grey-8')

        # Keep timer in page scope: NiceGUI forbids global UI elements with ui.page.
        ui.timer(1.0, update)


#################################################
#################### UTILS ######################
#################################################

def update():
    global time_left_label, score_label, sm_state_label, sm_state_desc_label, odom_label, speed_label, distance_label

    if ros_node.ready_to_start_match:
            if score_label is not None:
                score_label.set_text("0 points")

    if ros_node.match_started:
        ros_node.time_left = 100 - int(time.time() - ros_node.start_time)

        if time_left_label is not None:
            if ros_node.time_left > 0:
                time_left_label.set_text(f"{ros_node.time_left} secondes")
            else:
                time_left_label.set_text(f"{0} seconds")

        if score_label is not None:
            score_label.set_text(f'{ros_node.score} points')

    if sm_state_label is not None:
        sm_state_label.set_text(f'État: {ros_node.latest_sm_state}')

    if sm_state_desc_label is not None:
        sm_state_desc_label.set_text(ros_node.get_sm_state_description())

    if odom_label is not None:
        if ros_node.latest_odom_position is None:
            odom_label.set_text('Position: N/A')
        else:
            x, y = ros_node.latest_odom_position
            odom_label.set_text(f'Position: x={x:.3f} m, y={y:.3f} m')

    if speed_label is not None:
        if ros_node.latest_odom_velocity is None:
            speed_label.set_text('Vitesse: N/A')
        else:
            vx, vy = ros_node.latest_odom_velocity
            speed_label.set_text(f'Vitesse: {(vx**2 + vy**2)**0.5:.3f} m/s')

    if distance_label is not None:
        if ros_node.latest_odom_position is None or ros_node.latest_goal_position is None:
            distance_label.set_text('Distance au goal: N/A')
        else:
            ox, oy = ros_node.latest_odom_position
            gx, gy = ros_node.latest_goal_position
            distance_label.set_text(f'Distance au goal: {((gx-ox)**2+(gy-oy)**2)**0.5:.3f} m')

