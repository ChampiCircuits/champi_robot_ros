# Coupe de France de Robotique 2024 : Code ROS2

## Requirements

- Ubuntu 22
- ROS2 Iron
- Gazebo Harmonic

## How to

### Compile projects
```bash
colcon build --symlink-install
```
With `--symlink-install`, you can edit python files, launch files and config files without re-compiling.

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
