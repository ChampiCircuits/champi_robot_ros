import theme

from nicegui import ui, app

from node import init_ros_node
import time

ros_node = init_ros_node()

score_label = None
time_left_label = None
sm_state_label = None
odom_label = None



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
        global sm_state_label, odom_label
        with ui.column().style('position: fixed; bottom: 0; right: 0; z-index: 9999; align-items: flex-end; gap: 4px; padding: 8px;'):
            sm_state_label = ui.label('').classes('text-h6 text-grey-10')
            odom_label = ui.label('').classes('text-subtitle2 text-grey-8')

        # Keep timer in page scope: NiceGUI forbids global UI elements with ui.page.
        ui.timer(1.0, update)


#################################################
#################### UTILS ######################
#################################################

def update():
    global time_left_label, score_label, sm_state_label, odom_label

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

    if odom_label is not None:
        if ros_node.latest_odom_position is None:
            odom_label.set_text('Position (/odom): N/A')
        else:
            x, y = ros_node.latest_odom_position
            odom_label.set_text(f'Position (/odom): x={x:.3f}, y={y:.3f}')

