import tkinter as tk
from tkinter import ttk
import psutil
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

import subprocess
import netifaces
from signal import SIGINT

from math import acos, cos, sin
import rclpy
from nav_msgs.msg import Odometry


# Liste des fichiers de lancement ROS2
launch_files = [
    "champi_bringup bringup.launch.py sim:=True",
    "champi_nav2 bringup_launch.py",
    "file3.launch.py",
    "file4.launch.py",
    "file4.launch.py",
    "file4.launch.py",
    "file4.launch.py",

]

red = "#BC2023"
green = "#0C6B37"
white = "#FFFFFF"
orange = "#FFA500" 

class LaunchButton(tk.Button):
    def __init__(self, master, launch_file, **kwargs):
        super().__init__(master, **kwargs)
        self.launch_file = launch_file
        self.active = False
        self.process = None  # Initialisation du processus à None
        self.configure(bg=white)
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
        self.process = subprocess.Popen(["ros2", "launch", *args])
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

        self.tabControl = ttk.Notebook(self)

        self.tab1 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab1, text="LAUNCHS")

        self.tab2 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab2, text="MATCH")

        self.tab3 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab3, text="CPU")

        self.tab4 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab4, text="TABLE DE JEU")

        self.tab5 = ttk.Frame(self.tabControl)
        self.tabControl.add(self.tab5, text="IP")

        self.tabControl.pack(expand=1, fill="both")

        self.create_launchs_tab()
        self.create_cpu_tab()
        self.create_ip_tab()
        self.create_table_tab()

        # Ajout du gestionnaire d'événement pour la fermeture de la fenêtre
        self.protocol("WM_DELETE_WINDOW", self.close_window)

        rclpy.init()
        self.node = rclpy.create_node('GUI_node')
        self.odom_subscriber = self.node.create_subscription(Odometry, '/odom', self.update_robot_position, 10)

        self.update()

    def close_window(self):
        # kill all processes
        for child in self.launch_grid_frame.winfo_children():
            if isinstance(child, LaunchButton):
                child.kill_process()
        self.node.destroy_node()
        rclpy.shutdown()
        self.destroy()
        quit()

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
            button_text = f"Launch {launch_file}"
            button = LaunchButton(self.launch_grid_frame, text=button_text, launch_file=launch_file)
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

    def create_table_tab(self):
        table_frame = ttk.Frame(self.tab4)
        table_frame.pack(expand=True, fill="both")

        # Chemin de l'image
        image_path = "/home/etienne/ros_ws/src/champi_robot_ros/champi_simulator/blender_projects/table/table_dae/vinyle_table_2024_FINAL_V1.png"

        # Chargement de l'image
        self.table_image = tk.PhotoImage(file=image_path)

        self.table_image = self.table_image.subsample(17,17)
        
        # Création du canvas pour afficher l'image
        self.canvas = tk.Canvas(table_frame, width=self.table_image.width(), height=self.table_image.height())
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.table_image)


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


    def update(self):
        # tick ros
        rclpy.spin_once(self.node, timeout_sec=0.1)

        self.after(100, self.update)

        
if __name__ == "__main__":
    app = Application()
    app.mainloop()

    rclpy.shutdown()