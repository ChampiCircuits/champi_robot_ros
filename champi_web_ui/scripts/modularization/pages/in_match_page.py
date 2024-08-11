import theme

from nicegui import ui, app

from node import init_ros_node
from std_msgs.msg import Int32
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
                global score_label, time_left_label
                ui.label('Score :').classes('text-h4 text-grey-8')
                score_label = ui.label('0 points').classes('text-h4 text-black-8')

                ui.separator()

                ui.label('Time left :').classes('text-h4 text-grey-8')
                time_left_label = ui.label('100 secondes').classes('text-h4 text-black-8')
                # TODO link with the real time/points
                


#################################################
#################### UTILS ######################
#################################################

def update_time():
    global score, match_started, time_left_label, start_time, score_label

    if score == -1 and not match_started: # TODO faire mieux en publiant un Empty msg pour dire start
        print("\n\nRECEIVED MATCH STARTED\n\n")
        # match just started
        match_started = True
        start_time = time.time()
        score_label.set_text("0 points")

    if match_started:
        if 100 - int(time.time() - start_time) > 0:
            time_left_label.set_text(f"Temps restant: {100 - int(time.time() - start_time)} secondes")
        else:
            time_left_label.set_text(f"Temps restant: {0} secondes")


def update_score(received_score: Int32):
    global score_label, score
    score = received_score.data
    score_label.set_text(f'{score} points'.format(score=received_score.data))
    print(received_score)


score_subscriber = ros_node.create_subscription(Int32, '/final_score', update_score, 10)
ui.timer(1.0, update_time)