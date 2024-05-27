import psutil
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import tkinter as tk
from tkinter import ttk
import matplotlib

matplotlib.use("TkAgg")
# Création de la fenêtre principale
root = tk.Tk()
root.title("Utilisation du CPU et de la RAM en temps réel")

# Création de l'onglet
tab_control = ttk.Notebook(root)
tab3 = ttk.Frame(tab_control)
tab_control.add(tab3, text='Monitor')
tab_control.pack(expand=1, fill='both')

# Création du frame pour le graphe
cpu_frame = ttk.Frame(tab3)
cpu_frame.pack(expand=True, fill="both")

# Configuration du graphe
fig, ax = plt.subplots()
cpu_plot, = ax.plot([], [], label='CPU')
ram_plot, = ax.plot([], [], label='RAM')
ax.set_ylim(0, 100)
ax.set_xlim(0, 10)  # Limite initiale en x pour les 10 premiers points
ax.set_title('Utilisation du CPU et de la RAM en temps réel')
ax.set_xlabel('Temps (s)')
ax.set_ylabel('Utilisation (%)')
ax.legend()

# Test minimaliste : ajout d'une courbe de test initiale
ax.plot([0, 1, 2, 3], [0, 1, 4, 9], label='Test')
ax.legend()

# Initialiser les données
x_data = []
y_cpu_data = []
y_ram_data = []

# Fonction d'animation
def animate(i):
    cpu_percent = psutil.cpu_percent()
    ram_percent = psutil.virtual_memory().percent
    print(cpu_percent)

    # Mise à jour des données
    x_data.append(len(x_data))
    y_cpu_data.append(cpu_percent)
    y_ram_data.append(ram_percent)
    
    # Mise à jour des courbes
    cpu_plot.set_data(x_data, y_cpu_data)
    ram_plot.set_data(x_data, y_ram_data)
    
    # Ajustement dynamique de l'axe x
    ax.set_xlim(0, max(10, len(x_data)))
    return cpu_plot, ram_plot

ani = animation.FuncAnimation(fig, animate, interval=1000)

# Ajout du graphe à l'interface Tkinter
canvas = FigureCanvasTkAgg(fig, master=cpu_frame)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Lancement de la boucle principale
root.mainloop()
