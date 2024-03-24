# Coupe de France de Robotique 2024 : Code ROS2



Commandes


```bash
ros2 launch champi_bringup bringup.launch.py
```

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/arusso/dev/coupe/ws_0/src/champi_robot_ros/champi_nav2/config/map/map_test.yaml

/home/etienne/ros2_ws/src/champi_robot_ros/champi_nav2/config/map/map_test.yaml
```


Erreur TF from the past si le trajet dure plus de 10s.
Pour le résoudre, il y a un node spécial lancé acvec le launch file du robot.
Il ne faut pas utiliser Nav2 Goal sur rviz, mais l'autre.
https://github.com/ros-planning/navigation2/issues/3075
https://answers.ros.org/question/396864/nav2-computepathtopose-throws-tf-error-because-goal-stamp-is-out-of-tf-buffer/


## Requirements

- Ubuntu 22
- ROS2 Humble

```bash
sudo apt install ros-humble-twist-mux ros-humble-joy
```


## Commandements

Suis ces commandemments et tout se passera bien.

#### 1) Ne jamais créer de package python

Avec des packages c++, on peut faire aussi des nodes python. Mais avec des package python: 
- c'est super galère d'ajouter des nodes et des fichiers dans le projet
- bug avec symlink-install qui ne marche pas pour les launch files et les fichiers de config.
Regarder ce site pour savoir commment organiser un package c++ + python : https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

**Attention!** Ne pas oublier le sheebang `#!/usr/bin/env python3` en haut des nodes python! Sinon cela fait des erreurs atroces :(

## How to

### Configuration du projet

La première fois, se placer à la racine du workspace et lancer la commande suivante pour installer les dépendances et ajouter
des variables d'environnement et des alias dans le zshrc:
```bash
./src/champi_robot_ros/scripts/setup.sh
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

### U have an issue u don't understand :((
```bash
rm -R log build install # then recompile, and hope
```

## Nice ressources

* CAN cpp examples: https://github.com/craigpeacock/CAN-Examples/tree/master

## Notes

### setup.py deprecation warning

Pas de solution pour le moment : https://github.com/ament/ament_cmake/issues/382
