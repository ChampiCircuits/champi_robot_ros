#!/bin/sh

sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 65536
