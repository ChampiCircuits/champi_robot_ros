# Coupe de France de Robotique 2024 : Code ROS2

## Requirements

- Ubuntu 22
- ROS2 Humble

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

La première fois, se placer à la racine du workspace et lancer la commande suivante installer les packages ROS nécessaires:
```bash
rosdep install --from src --ignore-src
```
Et lancer le script `scripts/setup.sh` pour faire d'autres choses.

### Compilation
```bash
colcon build --symlink-install
```
With `--symlink-install`, you can edit python files, launch files and config files without re-compiling.

On peut aussi ajouter l'option `--packages-select mon_package` pour ne compiler que les package que l'on veut et ainsi réduire le temps de compilation.

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
