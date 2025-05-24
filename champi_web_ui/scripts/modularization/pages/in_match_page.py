import theme

from nicegui import ui, app

from node import init_ros_node
import time

ros_node = init_ros_node()

score_label = None
time_left_label = None



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


#################################################
#################### UTILS ######################
#################################################

def update():
    global time_left_label, score_label

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


ui.timer(1.0, update)