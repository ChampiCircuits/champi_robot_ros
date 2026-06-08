#!/bin/bash
set -ex

tmux new-session -d
tmux send-keys 'webots-controller VRACbotController.py' enter
sleep 0.1
tmux split-window -h
tmux send-keys 'python3 MotorBoard.py' enter
sleep 0.1
tmux split-window -h
tmux send-keys 'python3 RaspiBoard.py' enter
sleep 0.1
tmux split-window -h
tmux send-keys 'candump vcan0' enter
tmux select-layout even-vertical
tmux set-option mouse on
tmux set remain-on-exit on
tmux bind C-q kill-session
tmux attach-session

# close session with command: tmux kill-session
# close session with key bind: "Ctrl-b" then "Ctrl-q"
