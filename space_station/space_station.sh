#!/bin/bash

SESSION="ssos"

# Kill old session if exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? == 0 ]; then
    echo "Killing old tmux session: $SESSION"
    tmux kill-session -t $SESSION
fi

# Start new session (detached)
tmux new -s $SESSION -d -n eclss

# First window: ECLSS
tmux send-keys -t $SESSION:0 "ros2 launch space_station eclss.launch.py" C-m

# Second window: Thermals
tmux new-window -t $SESSION -n thermals
tmux send-keys -t $SESSION:1 "ros2 launch space_station thermals.launch.py" C-m

# Third window: GNC
tmux new-window -t $SESSION -n gnc
tmux send-keys -t $SESSION:2 "ros2 launch space_station gnc.launch.py" C-m

# Fourth window: GUI
tmux new-window -t $SESSION -n gui
tmux send-keys -t $SESSION:3 "ros2 run space_station space_station" C-m

# Attach to session
tmux attach -t $SESSION
