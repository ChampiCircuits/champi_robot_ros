#!/bin/bash

# Script set up in STM32CubeIDE to be called called before code generation

st_project_dir=$(pwd)

mv $st_project_dir/../Core/Src/main.cpp $st_project_dir/../Core/Src/main.c
mv $st_project_dir/../Core/Src/freertos.cpp $st_project_dir/../Core/Src/freertos.c

exit 0