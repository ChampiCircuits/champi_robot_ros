
from nicegui import ui
import plotly.graph_objects as go
from node import init_ros_node
from nav_msgs.msg import Odometry
from math import acos, sqrt
import theme

xy_fig, xy_plot = None, None
theta_fig, theta_plot = None, None


x_speeds = []
y_speeds = []
dir_speeds = []
thetas = []

last_x_speed = 0
last_y_speed = 0
last_dir_speed = 0
last_theta = 0

#################################################
#################### PAGE #######################
#################################################

def create() -> None:
    @ui.page('/speeds')
    def page_a():
        global xy_fig, xy_plot, theta_fig, theta_plot

        with theme.frame('Speeds Page'):

            with ui.column().style('width: 100%'):
                xy_fig = go.Figure()
                xy_fig.add_trace(go.Scatter(x=[], y=[], name="x_speed"))   # data[0]
                xy_fig.add_trace(go.Scatter(x=[], y=[], name="y_speed"))   # data[1]
                xy_fig.add_trace(go.Scatter(x=[], y=[], name="dir_speed")) # data[2]

                xy_fig.update_layout(margin=dict(l=0, r=0, t=0, b=0))
                xy_plot = ui.plotly(xy_fig).classes('w-full h-100')
                xy_plot.update()

                ui.label("theta")
                theta_fig = go.Figure()
                theta_fig.add_trace(go.Scatter(x=[], y=[], name="theta")) # data[0]

                theta_fig.update_layout(margin=dict(l=0, r=0, t=0, b=0))
                theta_plot = ui.plotly(theta_fig).classes('w-half h-30')
                theta_plot.update()

            ui.timer(1.0, update_plots)


        # ui.button('Add trace', on_click=add_trace)
#################################################
#################### UTILS ######################
#################################################

def vals_update(odom_msg):
    global last_x_speed, last_y_speed, last_dir_speed, last_theta
    last_x_speed, last_y_speed = odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y
    last_x_speed+=1
    last_y_speed-=1

    # oriented speed
    w = odom_msg.pose.pose.orientation.w
    last_theta = -2*acos(w)+1.57

    # Projection des vitesses x et y pour obtenir la vitesse absolue dans la direction theta
    last_dir_speed = sqrt(last_x_speed**2 + last_y_speed**2)



def update_plots():
    if xy_fig is None or xy_plot is None:
        return
    
    global x_speeds, y_speeds, dir_speeds, thetas

    x_speeds.append(last_x_speed)
    y_speeds.append(last_y_speed)
    dir_speeds.append(last_dir_speed)
    thetas.append(last_theta)


    xy_fig.data[0].update(x=list(range(1, len(x_speeds) + 1)), y=x_speeds)  # Update x_speeds trace
    xy_fig.data[1].update(x=list(range(1, len(y_speeds) + 1)), y=y_speeds)  # Update y_speeds trace
    xy_fig.data[2].update(x=list(range(1, len(dir_speeds) + 1)), y=dir_speeds)  # Update dir_speeds trace
    
    xy_plot.update()


    theta_fig.data[0].update(x=list(range(1, len(thetas) + 1)), y=thetas)  # Update theta trace

    theta_plot.update()



ros_node = init_ros_node()
ros_node.create_subscription(Odometry, '/odom', vals_update, 10)
