#!/bin/sh

mkdir -p out/python
mkdir -p out/cpp
mkdir -p out/nanopb

# Generate protobuf files in python and cpp
protoc --python_out=out/python/ *.proto
protoc --cpp_out=out/cpp/ *.proto

# Generate nanopb files
./protoc --nanopb_out=./out/nanopb/ *.proto
