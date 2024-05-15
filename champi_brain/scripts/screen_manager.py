#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import psutil
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from ament_index_python.packages import get_package_share_directory
import os

import subprocess
import netifaces
from signal import SIGINT

from math import acos, cos, sin
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
import time
from std_msgs.msg import Int32, String, Empty
from geometry_msgs.msg import Twist

from PIL import Image, ImageTk
from icecream import ic

from ament_index_python.packages import get_package_share_directory


red = "#BC2023"
green = "#0C6B37"
white = "#FFFFFF"
orange = "#FFA500"
blue = "#94C1CC"

# Liste des fichiers de lancement ROS2
launch_files = [
    "ros2 launch champi_bringup bringup.launch.py sim:=True",
    "ros2 launch champi_bringup bringup.launch.py sim:=False",
    "ros2 launch champi_nav2 bringup_launch.py",
    "",
    # "python3 src/champi_robot_ros/champi_brain/scripts/rviz_markers.py",
    # "ros2 launch champi_brain brain.launch.py",
    "python3 /home/champi/dev/ws_0/src/champi_robot_ros/scripts/kill_nodes.py",
    # ". /home/champi/dev/ws_0/src/champi_robot_ros/scripts/kill_nodes.sh",
    # "ros2 launch champi_brain brain.launch.py color:=homologation",
    "ros2 run champi_brain strategy_engine_node.py --ros-args -p color:=blue",
    # "ros2 launch champi_navigation control_pose_simulation.launch.py",
]

launch_colors = [
    white,
    blue,
    blue,
    white,
    white,
    blue
]



class ZoneButton(tk.Button):
    def __init__(self, parent, zone, node, color,x,y,table_img_height, **kwargs):
        w = int(0.45*table_img_height/2.0)
        h = w
 
        super().__init__(parent,width=w,height=h, **kwargs)
        self.zone = zone
        self.node = node
        self.configure(bg=color, command=self.toggle)
        # self.grid(row=row, column=column, sticky=sticky)
        parent.create_window( x,y,anchor = "nw", window = self,width=w,height=h) 
                    # button.grid(row=row, column=col, sticky=

    def toggle(self):
        print("button toggled : ",self.zone)
        msg = String()
        msg.data = self.zone
        self.node.zone_pub.publish(msg)

        self.node.final_score = 0
        self.node.match_started = False
        self.node.start_time = 0

class LaunchButton(tk.Button):
    def __init__(self, master, launch_file,color, **kwargs):
        super().__init__(master, **kwargs)
        self.launch_file = launch_file
        self.active = False
        self.process = None  # Initialisation du processus à None
        self.configure(bg=color)
        self.configure(command=self.toggle)
        self.has_been_stopped_by_user = False

    def toggle(self):
        if self.active:
            self.configure(bg=white)
            self.kill_process()
            self.has_been_stopped_by_user = True
            self.process = None
            self.active = False
        else:
            self.has_been_stopped_by_user = False
            self.configure(bg=green)
            self.launch_process()
            self.active = True

    def launch_process(self):
        args = map(str, self.launch_file.split(" "))
        self.process = subprocess.Popen([*args])
        self.after(1000, self.check_process)  # Vérifier l'état du processus toutes les 1 seconde


    def kill_process(self):
        if self.process and self.process.poll() is None:
            print("\nKILLING PROCESS {0} BY CLOSING WINDOW\n".format(self.launch_file))
            self.process.send_signal(SIGINT)
            try:
                print("Waiting for process to terminate")
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.configure(bg=orange)
                print("\n"*5)
                print("Timeout expired while waiting 30s for process to terminate. Killing it.")
                print("\n"*5)
                self.process.kill()
                self.process.wait()
                self.configure(bg=white)

            self.has_been_stopped_by_user = True

    def check_process(self):
        if self.process and self.process.poll() is not None and not self.has_been_stopped_by_user:  # Vérifier si le processus s'est terminé
            self.configure(bg=red)  # Mettre à jour l'état du bouton
            self.active = False  # Mettre à jour l'état du bouton
            self.process = None
            self.has_been_stopped_by_user = False
            print("Process: ", self.launch_file, " has stopped")
        else:
            self.after(1000, self.check_process)  # Réessayer après 1 seconde


class Application(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Champi Circuits Control Panel")
        self.geometry("800x600")

        style = ttk.Style()                     
        current_theme =style.theme_use()
        style.theme_settings(current_theme, {"TNotebook.Tab": {"configure": {"padding": [20, 20]}}}) 

        self.tabControl = ttk.Notebook(self)

        self.tab1 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab1, text="LAUNCHS")


        self.tab3 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab3, text="CPU")

        # self.tab4 = ttk.Frame(self.tabControl)
        # self.tabControl.add(self.tab4, text="TABLE DE JEU")

        self.tab5 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab5, text="IP")

        self.tab6 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab6, text="DIAGNOSTICS")

        self.tab7 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab7, text="CHOIX ZONE")

        self.tab2 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab2, text="MATCH")

        self.tabControl.pack(expand=1, fill="both")
    
        rclpy.init()
        self.node = rclpy.create_node('GUI_node')
        self.odom_subscriber = self.node.create_subscription(Odometry, '/odom', self.update_robot_position, 10)

        #score subscriber
        self.score_subscriber = self.node.create_subscription(Int32, '/final_score', self.update_score, 10)
        self.final_score = 0
        self.match_started = False
        self.start_time = 0

        self.zone_pub = self.node.create_publisher(String, '/start_zone', 10)
        self.tirette_pub = self.node.create_publisher(Empty, '/tirette_start', 10)


        # self.pub_STOP_FIN = self.node.create_publisher(Empty,'/STOP_FIN',10)
        self.pub_STOP_FIN = self.node.create_publisher(Twist,'/cmd_vel_stop',10)

        self.create_launchs_tab()
        self.create_cpu_tab()
        self.create_ip_tab()
        # self.create_table_tab()
        self.create_match_tab()
        self.create_choix_zone_tab()

        self.node_diagnostics = {}
        self.create_diagnostics_tab()

        # Ajout du gestionnaire d'événement pour la fermeture de la fenêtre
        self.protocol("WM_DELETE_WINDOW", self.close_window)

        self.update()

    def update_score(self, msg):
        print("\n\n----> Score received\n\n")
        self.final_score = msg.data
        # big big text, font size = 50
        self.score_label.config(text=f"Score: {self.final_score}")
        # self.score_label.config(font=("Courier", 50))

    def create_choix_zone_tab(self):
        # create a tab with 9 buttons in 3x3 grid
        # Couleurs et image pour les boutons
        button_colors = [
            '#477792', 'gray', '#CEAB47',
            '#CEAB47', 'red', '#477792',
            '#477792', 'gray', '#CEAB47'
        ]
        self.button_zones = [
            'B3', '', 'J2',
            'J3', 'T', 'B1',
            'B2', '', 'J1'
        ]
        self.start_zone = None


        arbre_file_path = os.path.join(get_package_share_directory('champi_brain'), 'scripts', 'arbre.png')

        # Charger l'image à utiliser pour le bouton central
        # self.image = Image.open("/home/champi/dev/ws_0/src/champi_robot_ros/champi_brain/scripts/arbre.png")
        self.image = Image.open(arbre_file_path)
        self.photo_image = ImageTk.PhotoImage(self.image)


        frame = ttk.Frame(self.tab7)
        frame.pack(expand=True, fill="both")

        # Chemin de l'image
        image_path = get_package_share_directory('champi_brain') + "/scripts/vinyle_table_2024_FINAL_V1.png"

        # Chargement de l'image
        self.table_image = tk.PhotoImage(file=image_path)

        self.table_image = self.table_image.subsample(17,17)

        # Création du canvas pour afficher l'image
        self.canvas = tk.Canvas(frame, width=self.table_image.width(), height=self.table_image.height())
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.table_image)

        # Création de la grille 3x3
        self.create_button_grid(self.canvas, 3, 3, button_colors)

    def tirette_start_pub(self):
        e = Empty()
        self.tirette_pub.publish(e)


    def create_button_grid(self, parent, rows, cols, button_colors):
        for row in range(rows):
            for col in range(cols):
                index = row * cols + col
                if button_colors[index] == 'red':
                    # Si le bouton doit être rouge, affiche l'image
                    button = tk.Button(parent, text="Tirette", command=self.tirette_start_pub, background=orange)
                    parent.create_window(
                        int(3/2*self.table_image.width()/3), 
                        int(1.5*self.table_image.height()/2),  
                        anchor = "center", 
                        window = button,
                        width=150,
                        height=70)
                    # button.grid(row=row, column=col, sticky="nsew")
                else:
                    # Sinon, utilisez la couleur donnée
                    # button = tk.Button(parent, bg=button_colors[index])
                    # button.configure(command=self.toggle_button(self.button_zones[index]))

                    if col == 0:
                        x = 0
                    elif col == 2:
                        x = (3-0.45)*self.table_image.width()/3

                    if row == 0:
                        y = 0
                    elif row == 1:
                        y = (1-0.45/2)*self.table_image.height()/2
                    elif row == 2:
                        y = (2-0.45)*self.table_image.height()/2

                    if col == 0 or col == 2:
                        button = ZoneButton(parent, self.button_zones[index], self, color=button_colors[index], x=x,y=y, table_img_height=self.table_image.height())
                        
                
                # Assurer que les colonnes et les rangées peuvent se développer
                # parent.columnconfigure(col, weight=1)
                # parent.rowconfigure(row, weight=1)


    def close_window(self):
        # kill all processes
        for child in self.launch_grid_frame.winfo_children():
            if isinstance(child, LaunchButton):
                child.kill_process()
        self.node.destroy_node()
        rclpy.shutdown()
        self.destroy()
        quit()

    def create_match_tab(self):
        # display score, font 50, centered in the window
        self.score_label = ttk.Label(self.tab2, text=f"Score: {self.final_score}",font=("Futura Gabriola Garamond", 50), justify=tk.CENTER,padding=(10,50)).pack()
        # display time left, font 50, centered in the window
        self.time_label = ttk.Label(self.tab2, text=f"Time left: 100",font=("Futura Gabriola Garamond", 50), justify=tk.CENTER,padding=(10,50)).pack()
        self.refresh_time()

    def refresh_time(self):
        # ic(self.match_started)
        # ic(int(time.time() - self.start_time))
        if self.final_score == -1 and not self.match_started:
            print("\n\nRECEIVED MATCH STARTED\n\n")
            # match just started
            self.match_started = True
            self.start_time = time.time()

        if self.match_started:
            if 100 - int(time.time() - self.start_time) > 0:
                self.time_label.config(text=f"Time left: {100 - int(time.time() - self.start_time)}")
            else:
                self.time_label.config(text=f"Time left: {0}")

                t = Twist()
                t.linear.x = 0.
                t.linear.y = 0.
                t.angular.z = 0.
                self.pub_STOP_FIN.publish(t)
                
        self.after(1000, self.refresh_time)
        
    def create_launchs_tab(self):
        self.launch_grid_frame = ttk.Frame(self.tab1)
        self.launch_grid_frame.pack(expand=True, fill="both")

        row = 0
        col = 0
        for i, launch_file in enumerate(launch_files):
            if i % 2 == 0 and i != 0:
                row += 1
                col = 0
            self.launch_grid_frame.grid_rowconfigure(row, weight=1)
            self.launch_grid_frame.grid_columnconfigure(col, weight=1)  # Ajout de la configuration de la colonne
            button_text = f"{launch_file}"
            button = LaunchButton(self.launch_grid_frame, text=button_text, launch_file=launch_file, color=launch_colors[i])
            button.grid(row=row, column=col, sticky="nsew")  # Utilisation de sticky pour prendre toute la place
            col += 1

    def create_cpu_tab(self):
        cpu_frame = ttk.Frame(self.tab3)
        cpu_frame.pack(expand=True, fill="both")

        fig, ax = plt.subplots()
        cpu_plot, = ax.plot([], [], label='CPU')
        ram_plot, = ax.plot([], [], label='RAM')
        ax.set_ylim(0, 100)
        ax.set_xlim(0, 100)
        ax.set_title('Utilisation du CPU et de la RAM en temps réel')
        ax.set_xlabel('Temps (s)')
        ax.set_ylabel('Utilisation (%)')
        ax.legend()

        def animate(i):
            cpu_percent = psutil.cpu_percent()
            ram_percent = psutil.virtual_memory().percent
            x_data = list(range(len(cpu_plot.get_xdata()) + 1))
            y_cpu_data = list(cpu_plot.get_ydata())
            y_ram_data = list(ram_plot.get_ydata())
            y_cpu_data.append(cpu_percent)
            y_ram_data.append(ram_percent)
            cpu_plot.set_data(x_data, y_cpu_data)
            ram_plot.set_data(x_data, y_ram_data)
            ax.set_xlim(0, x_data[-1] + 1)
            return cpu_plot, ram_plot

        ani = animation.FuncAnimation(fig, animate, interval=1000)

        canvas = FigureCanvasTkAgg(fig, master=cpu_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def create_ip_tab(self):
        # Votre code pour l'onglet "IP"
        ip_frame = ttk.Frame(self.tab5)
        ip_frame.pack(expand=True, fill="both")

        self.ip_label = ttk.Label(ip_frame, text="\n\nAdresse IP:")
        self.ip_label.pack()

        self.ip_address = ttk.Label(ip_frame, text=self.get_ip_addresses())
        self.ip_address.pack()

        self.wifi_label = ttk.Label(ip_frame, text="\n\nNom du réseau WiFi actuel:")
        self.wifi_label.pack()

        self.wifi_name = ttk.Label(ip_frame, text=self.get_wifi_name())
        self.wifi_name.pack()

        # Bouton Actualiser
        refresh_button = ttk.Button(ip_frame, text="Actualiser", command=self.refresh_ip_tab)
        refresh_button.pack()

    def refresh_ip_tab(self):
        self.ip_address.config(text=self.get_ip_addresses())
        self.wifi_name.config(text=self.get_wifi_name())

    def get_ip_address(self,interface):
        try:
            addrs = netifaces.ifaddresses(interface)
            ip_info = {}
            if netifaces.AF_INET in addrs:  # Check for IPv4 address
                ip_info['IPv4'] = addrs[netifaces.AF_INET][0]['addr']
            if netifaces.AF_INET6 in addrs:  # Check for IPv6 address
                ip_info['IPv6'] = addrs[netifaces.AF_INET6][0]['addr']
            if not ip_info:
                return "Not connected"
            return ip_info['IPv4']
        except ValueError:
            return {}

    def get_ip_addresses(self):
        interfaces = netifaces.interfaces()

        ip_addresses = []
        for interface_name in interfaces:
            ip_address = self.get_ip_address(interface_name)
            ip_addresses.append(f"{interface_name}: {ip_address}")

        return "\n".join(ip_addresses)

    def get_wifi_name(self):
        subprocess_result = subprocess.Popen('iwgetid',shell=True,stdout=subprocess.PIPE)
        subprocess_output = subprocess_result.communicate()[0],subprocess_result.returncode
        network_name = subprocess_output[0].decode('utf-8')
        return network_name

    # def create_table_tab(self):
    #     table_frame = ttk.Frame(self.tab4)
    #     table_frame.pack(expand=True, fill="both")

    #     # Chemin de l'image
    #     image_path = get_package_share_directory('champi_brain') + "/scripts/vinyle_table_2024_FINAL_V1.png"

    #     # Chargement de l'image
    #     self.table_image = tk.PhotoImage(file=image_path)

    #     self.table_image = self.table_image.subsample(17,17)

    #     # Création du canvas pour afficher l'image
    #     self.canvas = tk.Canvas(table_frame, width=self.table_image.width(), height=self.table_image.height())
    #     self.canvas.pack()
    #     self.canvas.create_image(0, 0, anchor=tk.NW, image=self.table_image)

    def update_robot_position(self,odom_msg):
        # erase previous position
        # delete all but the image
        for item in self.canvas.find_all():
            if item != 1:
                self.canvas.delete(item)

        x,y,z,w = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
        theta = -2*acos(w)+1.57

        y_pixel = int((x / 2) * self.table_image.height())
        x_pixel = int((y / 3) * self.table_image.width())

        r=7
        self.canvas.create_oval(x_pixel - r, y_pixel - r, x_pixel + r, y_pixel + r, fill="red")

        r=int(0.3/2/3.0*self.table_image.width())
        self.canvas.create_oval(x_pixel - r, y_pixel - r, x_pixel + r, y_pixel + r, outline="green", width=4)

        # ligne en fonction de theta
        self.canvas.create_line(x_pixel, y_pixel, x_pixel + r *cos(theta), y_pixel + r *sin(theta), fill="red", width=4)
        self.canvas.create_line(x_pixel, y_pixel, x_pixel + r *cos(theta-1.57), y_pixel + r *sin(theta-1.57), fill="green", width=4)


        self.canvas.create_oval(0 - r, 0 - r, 0 + r, 0 + r, outline="green", width=4)
        self.canvas.create_oval(2 - r, 3 - r, 2 + r, 3 + r, outline="green", width=4)

    def create_diagnostics_tab(self):
        self.diagnostics_frame = ttk.Frame(self.tab6)
        self.diagnostics_frame.pack(expand=True, fill="x")

        self.diagnostics_texts = {}  # Dictionnaire pour stocker les éléments Text correspondant à chaque diagnostic

        # Souscription au topic des diagnostics
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.update_diagnostics, 10)

    def update_diagnostics(self, msg):
        current_time = time.time()
        # Mettre à jour le dictionnaire avec les nouveaux diagnostics
        for status in msg.status:
            # Vérifier si le diagnostic existe déjà dans le dictionnaire
            name = status.name[status.name.find(':')+1:]
            if name in self.node_diagnostics:
                # Mettre à jour le diagnostic et son horodatage
                self.node_diagnostics[name]['message'] = status.message
                self.node_diagnostics[name]['timestamp'] = current_time
                self.node_diagnostics[name]['level'] = status.level
                # si le msg contient "inactive" on change le statut
                if "inactive" in status.message:
                    print("inactiiiiiiiive")
                    self.node_diagnostics[name]['level'] = b'\x02'
                else:
                    self.node_diagnostics[name]['level'] = b'\x00'
            else:
                # Ajouter un nouveau diagnostic avec son horodatage
                self.node_diagnostics[name] = {'message': status.message, 'timestamp': current_time, 'level': status.level}

        # Effacer le contenu précédent
        for text in self.diagnostics_texts.values():
            text.delete("1.0", tk.END)

        # Mettre à jour l'affichage des diagnostics à partir du dictionnaire
        for node_name, diagnostic_info in self.node_diagnostics.items():
            diagnostic_message = diagnostic_info['message']
            diagnostic_level = diagnostic_info['level']
            color = self.get_color_for_diagnostic_level(diagnostic_level)

            # Vérifier si un élément Text existe déjà pour ce diagnostic, sinon le créer
            if node_name not in self.diagnostics_texts:
                self.diagnostics_texts[node_name] = tk.Text(self.diagnostics_frame, wrap="word", background=color)
                self.diagnostics_texts[node_name].pack(fill="x", expand=True)
                self.diagnostics_texts[node_name].config(height=3)  # Ajustez la hauteur selon vos besoins

            # Insérer le message du diagnostic dans l'élément Text correspondant
            self.diagnostics_texts[node_name].insert(tk.END, str(node_name) + "\t"+str(diagnostic_message)+"\n")

    def invalidate_old_diagnostics(self):
        current_time = time.time()
        # Mettre à jour l'affichage des diagnostics à partir du dictionnaire
        for node_name, diagnostic_info in self.node_diagnostics.items():
            # Vérifier si le diagnostic est toujours valide
            if current_time - diagnostic_info['timestamp'] > 10:
                self.node_diagnostics[node_name]['message'] = "Inactive (10s+)"
                self.node_diagnostics[node_name]['level'] = b'\x02'  # ERROR

        for text in self.diagnostics_texts.values():
            text.delete("1.0", tk.END)

        # Mettre à jour l'affichage des diagnostics à partir du dictionnaire
        for node_name, diagnostic_info in self.node_diagnostics.items():
            diagnostic_message = diagnostic_info['message']
            color = self.get_color_for_diagnostic_level(diagnostic_info['level'])
            # Vérifier si un élément Text existe déjà pour ce diagnostic, sinon le créer
            if node_name not in self.diagnostics_texts:
                self.diagnostics_texts[node_name] = tk.Text(self.diagnostics_frame, wrap="word", background=color)
                self.diagnostics_texts[node_name].pack(fill="x", expand=True)

            # Insérer le message du diagnostic dans l'élément Text correspondant
            self.diagnostics_texts[node_name].insert(tk.END, str(node_name) + "\t"+str(diagnostic_message)+"\n")

    def get_color_for_diagnostic_level(self, level):
        if level == b'\x00':  # OK
            return "green"
        elif level == b'\x01':  # WARN
            return "orange"
        elif level == b'\x02':  # ERROR
            return "red"
        else:  # Autres niveaux non définis
            print("level not defined: ", level)
            return "black"  # ou une autre couleur par défaut

    def update(self):
        # tick ros
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # self.invalidate_old_diagnostics()

        self.after(100, self.update)

    class DiagnosticsNode(Node):
        def __init__(self):
            super().__init__('diagnostics_node')

if __name__ == "__main__":
    app = Application()
    app.mainloop()

    rclpy.shutdown()
