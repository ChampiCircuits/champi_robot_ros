#!/bin/bash

if ! tmux has-session -t champi_session 2>/dev/null; then
  sudo systemctl stop champi.service
  sudo systemctl start champi.service
fi

tmux attach -t champi_session