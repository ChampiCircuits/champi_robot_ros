#!/bin/bash

sudo systemctl stop champi.service
sudo systemctl start champi.service
tmux attach -t champi_session