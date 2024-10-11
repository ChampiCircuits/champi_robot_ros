import theme
import subprocess

from nicegui import ui

# Liste des fichiers de lancement ROS2
launch_dic = [
    {"text":"bringup SIM ","command":"ros2 launch champi_bringup bringup.launch.py sim:=True", "color":"blue"},
    {"text":"bringup REAL","command":"ros2 launch champi_bringup bringup.launch.py sim:=False", "color":"blue"},
    {"text":"KILL NODES","command":"python3 /home/champi/dev/ws_0/src/champi_robot_ros/scripts/kill_nodes.py", "color":"red"},
    {"text":"strategy engine","command":"ros2 run champi_brain strategy_engine_node.py --ros-args -p color:=blue", "color":"blue"},
    {"text":"","command":"", "color":"blue"},
]

#################################################
#################### PAGE #######################
#################################################
class LaunchButton(ui.button):
    def __init__(self, command, text, color, *args, **kwargs) -> None:
        super().__init__(*args, *kwargs)
        self.on('click', self.click)
        self.command = command
        self._text = text
        self.props(f"color={color}")

    def click(self) -> None:
        print(self.command)
        args = map(str, self.command.split(" "))
        self.process = subprocess.Popen([*args])


def create() -> None:
    @ui.page('/')
    def page_launchs():
        with theme.frame('Launch Page'):
            with ui.grid(columns=3).style('width: 90%'):
                for launch in launch_dic:
                    LaunchButton(command=launch["command"], text=launch["text"], color=launch["color"])


#################################################
#################### UTILS ######################
#################################################



# TODO verif l'etat des process lancés comme avec screen_manager ?
# afficher les logs direct en dessous des boutons ?
# en faire des toggle pour pouvoir les kill