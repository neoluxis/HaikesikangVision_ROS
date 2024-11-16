#!/bin/bash

cd /root/dev_ws/appli
source install/setup.bash

tmux new-session -d -s appli

tmux split-window -h
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "appli.sh" C-m

tmux select-pane -t 2
tmux send-keys "sleep 8" C-m
tmux send-keys "ros2 topic echo /serial_send" C-m

tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux attach -t appli
