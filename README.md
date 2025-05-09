# Coupe de France de Robotique 2024 : Code ROS2

sudo systemctl restart isc-dhcp-server

## Pictures

|2024 robot!|
| -------- |
|<img src="docs/ressources/robot1.jpg"  width="50%">|



## Links to other READMEs

* [marker_helper](champi_libraries_py/champi_libraries_py/marker_helper/README.md)

## Setup

1) Make sure your workspace is `~/champi_ws`. The scripts are hardcoded to this path.
2) Install dependencies:
```bash
~/champi_ws/src/champi_robot_ros/setup/install_deps.sh
```
3) Setup the environment (exports, aliases, etc.) by adding one of those line in your `.bashrc` or `.zshrc`:
```bash
source ~/champi_ws/src/champi_robot_ros/setup/env/champi_env_dev_pc.sh # on your PC
# or
source ~/champi_ws/src/champi_robot_ros/setup/env/champi_env_robot.sh # on the robot 
```

3) Every time you want to share your internet connection with the robot, run:
```bash
share_internet
```

## Using Clion

When opening the project, a lot of build directories are displayed in the project tree.
Remove them by un-ticking `clion.workspace.external.source.group.into.folders` in Registry (Search in Shift Shift menu).


## Robot

### IPs

* Over Wifi (hotspot): `172.0.0.1`
* Over direct Ethernet: `10.0.0.1`

## Commandes

### Teleop
```bash
ros2 launch champi_bringup teleop.launch.py
```

*Paramètres*:
- *sim* : `true` | `false`.
- *joy* : `true` | `false`.

### Navigation
```bash

```



## Requirements

- Ubuntu 24
- ROS2 Jazzy

## How to


### STM32

Create a symlink to the STM32 project in the workspace:
```bash
ln -s /ros2_ws/stm_main_board /stm_ws/stm_main_board
```

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

## CAN
### Generate protobuf files for the CAN Bus
Add the msg in the .proto file, and then :
```bash
cd scripts/gen_proto
chmod +x gen.sh
./gen.sh
```
@See the comment  // comment me when compile for cpp and python


### Generate Msg IDs for the CAN Bus
Add the msg id in the .csv file, and then :
```bash
cd scripts/gen_can_ids
python3 gen_ids.py
```

### Replace by the generated files :
- from gen_proto/out/
    - champi_libraries_cpp/include/champi_can/msgs_can.pb.h
    - champi_libraries_cpp/src/champi_can/msgs_can.pb.cc
- from gen_can_ids/out/can_ids_ns.hpp **(copy content from NS file)**
    - champi_libraries_cpp/include/champi_can/can_ids.hpp

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
