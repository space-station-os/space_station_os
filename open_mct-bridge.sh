#!/bin/bash

SESSION="openmct_rosbridge"
OPENMCT_DIR="$HOME/ssos_ws/src/space_station_os/space_station_mission_control/openmct-ros"

# Kill if session already exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? -eq 0 ]; then
  echo "Killing existing tmux session: $SESSION"
  tmux kill-session -t $SESSION
fi

# Create new tmux session and window
tmux new-session -d -s $SESSION -n main

# Pane 1: Launch rosbridge_websocket
tmux send-keys -t $SESSION 'source ~/ssos_ws/install/setup.bash' C-m
tmux send-keys -t $SESSION 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' C-m

# Split window horizontally and run OpenMCT
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $OPENMCT_DIR" C-m
tmux send-keys -t $SESSION 'npm start' C-m

# Attach to the session
tmux attach-session -t $SESSION
