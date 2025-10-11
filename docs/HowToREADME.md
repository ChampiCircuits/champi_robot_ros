# How to

## STM32

Create a symlink to the STM32 project in the workspace:
```shell
ln -s /ros2_ws/stm_main_board /stm_ws/stm_main_board
```

## Create python Node

**Warning!** Never forget the sheebang `#!/usr/bin/env python3` as the first line of python nodes. Otherwise horrible to understand errors will appear :(

## Create a package: never create a python package !

With C++ packages, you can also create Python nodes. But with Python packages: 
- it's a real pain to add nodes and files to the project
- there's a bug with symlink-install that doesn't work for launch files and config files.
Check out this site to learn how to organize a C++ + Python package: https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/


## Publish goals for nav2 with RViz2

Use “2D Nav Goal” to publish a goal, not “Nav2 Goal.”

These two buttons publish a goal with a timestamp. When nav2 searches for the transformation between the goal and the robot's current pose, it looks at the goal from the past (because of its timestamp). If the path takes more than 10 seconds, it exceeds the 10-second TF buffer.
There is therefore an additional node launched by champi_bringup that republishes “2D Nav Goal” by setting the timestamp to 0, which allows nav2 to use the latest available transformation.
See the following links for more information:

Translated with DeepL.com (free version)

https://github.com/ros-planning/navigation2/issues/3075

https://answers.ros.org/question/396864/nav2-computepathtopose-throws-tf-error-because-goal-stamp-is-out-of-tf-buffer/






_____________________
_____________________
_____________________
_____________________
_____________________


# !! OUTDATED !!
## CAN
* CAN cpp examples: https://github.com/craigpeacock/CAN-Examples/tree/master

### Generate protobuf files for the CAN Bus
Add the msg in the .proto file, and then :
```shell
cd scripts/gen_proto
chmod +x gen.sh
./gen.sh
```
@See the comment  // comment me when compile for cpp and python


### Generate Msg IDs for the CAN Bus
Add the msg id in the .csv file, and then :
```shell
cd scripts/gen_can_ids
python3 gen_ids.py
```

### Replace by the generated files :
- from gen_proto/out/
    - champi_libraries_cpp/include/champi_can/msgs_can.pb.h
    - champi_libraries_cpp/src/champi_can/msgs_can.pb.cc
- from gen_can_ids/out/can_ids_ns.hpp **(copy content from NS file)**
    - champi_libraries_cpp/include/champi_can/can_ids.hpp
