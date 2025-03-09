#!/bin/bash

# Script set up in STM32CubeIDE to be called called after code generation

st_project_dir=$(pwd)

mv $st_project_dir/../Core/Src/main.c $st_project_dir/../Core/Src/main.cpp
mv $st_project_dir/../Core/Src/freertos.c $st_project_dir/../Core/Src/freertos.cpp
