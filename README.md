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

## Notes

### setup.py deprecation warning

Pas de solution pour le moment : https://github.com/ament/ament_cmake/issues/382
