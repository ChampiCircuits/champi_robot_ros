#!/bin/sh

mkdir -p out/python
mkdir -p out/cpp
mkdir -p out/nanopb

echo "compile for cpp and python"

# Generate protobuf files in python and cpp
protoc --python_out=out/python/ *.proto
protoc --cpp_out=out/cpp/ *.proto

echo "generate nanopb files"

# Generate nanopb files
./nano_pb_generator/protoc --nanopb_out=./out/nanopb/ *.proto
