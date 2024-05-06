# Coupe de France de Robotique 2024 : Code ROS2

## Commandes


### Robot
#### boot de la rpi
 ./dev/ws_0/src/champi_robot_ros/scripts/bringup-can0.sh


### MATCH
PLUS qu'à lancer :
python3 /home/andre/dev/coupe/ros_ws/src/champi_robot_ros/champi_brain/scripts/screen_manager.py


SINON en séparé :

```bash
ros2 launch champi_bringup bringup.launch.py sim:=False
ros2 launch champi_nav2 bringup_launch.py
ros2 launch champi_brain brain.launch.py color:=blue
OU
ros2 launch champi_brain brain.launch.py color:=yellow
```


### si besoin
```bash
. /home/champi/dev/ws_0/src/champi_robot_ros/scripts/kill_nodes.sh
```

*Paramètres*:
- *sim* : `true` | `false`.
- *joy* : `true` | `false`.

### Teleop
```bash
ros2 launch bringup teleop.launch.py
```

*Paramètres*:
- *sim* : `true` | `false`.
- *joy* : `true` | `false`.

### Navigation
```bash
ros2 launch champi_nav2 bringup_launch.py
```
```bash
python3 dev_tools/goals_cmd.py
```


## Requirements

- Ubuntu 22
- ROS2 Humble

## How to

### Configuration du projet

La première fois, se placer à la racine du workspace et lancer la commande suivante pour installer les dépendances et ajouter
des variables d'environnement et des alias dans le zshrc:
```bash
./src/champi_robot_ros/scripts/setup.sh
```
depuis le workspace :
```bash
rosdep install --from-paths src -y --ignore-src 
```

### Compilation

Le script `setup.sh` a ajouté des alias dans le zshrc pour simplifier la compilation: on peut lancer la commande
`champi_build` depuis n'importe quel répertoire pour compiler le workspace.

On peut aussi ajouter des options à la commande `champi_build`. Celles-ci sont les mêmes que pour `colcon build`.
Par exemple, pour ne compiler que le package `mon_package`:
```bash
champi_build --packages-select mon_package
```

### Generate protobuf files for the CAN Bus
```bash
cd non_ros/gen_proto
chmod +x gen.sh
./gen.sh
```

### Publier des goals pour nav2 sur Rviz2

Utiliser "2D Nav Goal" pour publier un goal, pas "Nav2 Goal".

En effet ces 2 boutons publient un goal avec timestamp. Lorsque nav2 cherche la transformation entre le goal et la pose actuelle du robot, il regarde donc par rapport au goal du passé (à cause de sa timestamp). Si le trajet fait plus de 10s, il dépasse le buffer TF de 10s.
Il y a donc un node supplémentaire lancé par champi_bringup qui republie " 2D Nav Goal" en mettant à 0 le timestamp, ce qui permet à nav2 d'utiliser la dernière transformation disponible.
Voir les liens suivants pour plus d'infos:

https://github.com/ros-planning/navigation2/issues/3075

https://answers.ros.org/question/396864/nav2-computepathtopose-throws-tf-error-because-goal-stamp-is-out-of-tf-buffer/


###  Créer un package: ne jamais créer de package python !

Avec des packages c++, on peut faire aussi des nodes python. Mais avec des package python: 
- c'est super galère d'ajouter des nodes et des fichiers dans le projet
- bug avec symlink-install qui ne marche pas pour les launch files et les fichiers de config.
Regarder ce site pour savoir commment organiser un package c++ + python : https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

**Attention!** Ne pas oublier le sheebang `#!/usr/bin/env python3` en haut des nodes python! Sinon cela fait des erreurs atroces :(

## Nice ressources

* CAN cpp examples: https://github.com/craigpeacock/CAN-Examples/tree/master

## Notes

### setup.py deprecation warning

Pas de solution pour le moment : https://github.com/ament/ament_cmake/issues/382
