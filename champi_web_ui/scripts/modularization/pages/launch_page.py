import theme
import subprocess

from nicegui import ui

# Liste des fichiers de lancement ROS2
launch_dic = [
    {"text":"chapi ","command":"chapi", "color":"blue"},
    {"text":"chapo ","command":"chapo", "color":"blue"},
    {"text":"launch brain","command":"ros2 launch champi_brain brain.launch.py", "color":"blue"},
    {"text":"KILL NODES","command":"kill_nodes", "color":"red"},
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