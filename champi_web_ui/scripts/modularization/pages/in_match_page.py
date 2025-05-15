import theme

from nicegui import ui, app

from node import init_ros_node
from std_msgs.msg import Int8
import time

ros_node = init_ros_node()

score = 0
time_left = 100
score_label = None
time_left_label = None
match_started = False
start_time = 0



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
                score_label = ui.label(str(score) + ' points').classes('text-h4 text-black-8')

                ui.separator()

                ui.label('Temps restant :').classes('text-h4 text-grey-8')
                time_left_label = ui.label(str(time_left) + ' secondes').classes('text-h4 text-black-8')            


#################################################
#################### UTILS ######################
#################################################

def update():
    global score, match_started, time_left_label, start_time, score_label

    if not match_started:
        if ros_node.last_stm_state is not None and ros_node.last_stm_state.tirette_released:
            print("\n\n!! MATCH STARTED !!\n\n")

            match_started = True
            start_time = time.time()
            if score_label is not None:
                score_label.set_text("0 points")

    if match_started:
        if time_left_label is not None:
            if 100 - int(time.time() - start_time) > 0:
                time_left_label.set_text(f"{100 - int(time.time() - start_time)} secondes")
            else:
                time_left_label.set_text(f"{0} seconds")


def update_score(received_score: Int8):
    global score_label, score
    score = received_score.data
    if score_label is not None:
        score_label.set_text(f'{score} points'.format(score=received_score.data))
    print("received score= ",received_score)


score_subscriber = ros_node.create_subscription(Int8, '/final_score', update_score, 10)
ui.timer(1.0, update)