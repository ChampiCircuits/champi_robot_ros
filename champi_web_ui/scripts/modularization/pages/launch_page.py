import theme
import subprocess
import os

from nicegui import ui

# Liste des fichiers de lancement ROS2
launch_dic = [
    # {"text":"chapi ","command":"~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_start.bash", "color":"blue"},
    # {"text":"chapo ","command":"~/champi_ws/src/champi_robot_ros/setup/robot/tmux/champi_stop.bash", "color":"blue"},
    # {"text":"launch brain","command":"ros2 launch champi_brain brain.launch.py", "color":"blue"},
    # {"text":"KILL NODES","command":"kill_nodes", "color":"red"},
    # {"text":"open terminal","command":"gnome-terminal", "color":"orange"},
    {"text":" restart bringup (champi.service = chapi)","command":"sudo systemctl restart champi.service", "color":"orange"},
    {"text":" restart system (sensors+web+foxglove)","command":"sudo systemctl restart champystem.service", "color":"orange"},

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


#################################################
################ SERVICE STATUS #################
#################################################

MONITORED_SERVICES = ['champi.service', 'champystem.service']


def get_service_status(service_name: str) -> str:
    try:
        result = subprocess.run(
            ['systemctl', 'show', '-p', 'SubState', '--value', service_name],
            capture_output=True, text=True, timeout=3
        )

        return result.stdout.strip()
    except Exception:
        return 'unknown'


def status_color(status: str) -> str:
    return {'running': 'text-green-600', 'inactive': 'text-red-600'}.get(status, 'text-orange-500')


def get_tmux_sessions() -> str:
    try:
        result = subprocess.run(
            ['tmux', 'ls'],
            capture_output=True, text=True, timeout=3
        )
        output = result.stdout.strip()
        return output if output else '(no sessions)'
    except FileNotFoundError:
        return 'tmux not found'
    except Exception:
        return 'error'


def create() -> None:
    @ui.page('/launchs')
    def page_launchs():
        with theme.frame('Launch Page'):
            ui.label('Service status').classes('text-lg font-bold mt-4')
            with ui.grid(columns=2).style('width: 60%; gap: 4px'):
                for service in MONITORED_SERVICES:
                    ui.label(service).classes('font-mono')
                    status_label = ui.label(get_service_status(service)).classes('font-bold')
                    # capture per-iteration bindings
                    def make_updater(lbl, svc):
                        def update():
                            status = get_service_status(svc)
                            lbl.set_text(status)
                            lbl.classes(
                                status_color(status),
                                remove='text-green-600 text-red-600 text-orange-500'
                            )
                        return update
                    ui.timer(2.0, make_updater(status_label, service))
                    # set initial color
                    make_updater(status_label, service)()

            ui.separator().style('margin: 16px 0')
            ui.label('Tmux sessions').classes('text-lg font-bold')
            tmux_label = ui.label(get_tmux_sessions()).classes('font-mono whitespace-pre')

            def update_tmux():
                tmux_label.set_text(get_tmux_sessions())
            ui.timer(2.0, update_tmux)

            ui.separator().style('margin: 16px 0')
            with ui.grid(columns=3).style('width: 90%'):
                for launch in launch_dic:
                    LaunchButton(command=launch["command"], text=launch["text"], color=launch["color"])


#################################################
#################### UTILS ######################
#################################################



# TODO verif l'etat des process lancés comme avec screen_manager ?
# afficher les logs direct en dessous des boutons ?
# en faire des toggle pour pouvoir les kill