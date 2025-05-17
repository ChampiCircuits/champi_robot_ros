import theme
import subprocess
import os

from nicegui import ui

# Liste des fichiers de lancement ROS2
launch_dic = [
    # {"text":"chapi ","command":"~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_start.bash", "color":"blue"},
    # {"text":"chapo ","command":"~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_stop.bash", "color":"blue"},
    {"text":"launch brain","command":"ros2 launch champi_brain brain.launch.py", "color":"blue"},
    # {"text":"KILL NODES","command":"kill_nodes", "color":"red"},
    # {"text":"open terminal","command":"gnome-terminal", "color":"orange"},
    {"text":" restart isc-dhcp-server","command":"sudo systemctl restart isc-dhcp-server", "color":"orange"},
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

        display = ":0"  # :0 est souvent l'affichage principal
        env = os.environ.copy()
        sudo_password = 'circuits'
        env["DISPLAY"] = display
        uid = os.getuid()
        env["XDG_RUNTIME_DIR"] = f"/run/user/{uid}"

        # Prépare la commande à exécuter avec sudo
        full_command = f"echo {sudo_password} | sudo -S {self.command}"

        try:
            # Utilise subprocess pour exécuter la commande
            process = subprocess.Popen(full_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)

            # Attente de la fin du processus
            stdout, stderr = process.communicate()

            # Affiche les résultats
            if process.returncode == 0:
                print(f"Commande exécutée avec succès :\n{stdout.decode()}")
            else:
                print(f"Erreur lors de l'exécution de la commande :\n{stderr.decode()}")

        except Exception as e:
            print(f"Erreur: {str(e)}")



def create() -> None:
    @ui.page('/launchs')
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