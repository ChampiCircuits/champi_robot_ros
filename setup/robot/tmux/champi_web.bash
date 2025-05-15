#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/champi/champi_ws/install/setup.bash

echo "Attente avant le démarrage complet du pc..."
sleep 15

echo "Lancement du serveur web sur le port 8080 !"
python3 ~/champi_ws/src/champi_robot_ros/champi_web_ui/scripts/modularization/main.py &

sleep 2

# Lancer Firefox dans la session graphique en mode kiosk
echo "Lancement de Firefox !"
#XDG_RUNTIME_DIR=/run/user/$(id -u) systemd-run --user firefox --kiosk http://localhost:8080
XDG_RUNTIME_DIR=/run/user/$(id -u) systemd-run --user firefox --kiosk http://localhost:8080

# Attendre que Firefox se ferme
while pgrep firefox > /dev/null; do
    sleep 10  # Vérifie toutes les 10 secondes si Firefox est toujours en cours d'exécution
done