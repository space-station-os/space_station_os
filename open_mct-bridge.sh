#!/bin/bash

SESSION="openmct_rosbridge"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_DIR="$HOME/ssos_ws"

if [[ "$SCRIPT_DIR" == "$DEFAULT_DIR"* ]]; then
  WS_DIR="$DEFAULT_DIR"
elif [ -n "$1" ]; then
  WS_DIR="$1"
else
  read -p "Enter your ROS 2 workspace directory (e.g. ~/ros2_ws): " WS_DIR
fi

WS_DIR=$(eval echo "$WS_DIR")

INSTALL_DIR="$WS_DIR/install"
OPENMCT_DIR="$(find "$WS_DIR/src" -type d -name openmct-ros | head -n 1)"
echo "[INFO] Ensuring npm dependencies are installed..."
cd "$OPENMCT_DIR"
npm install

# Kill if session already exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? -eq 0 ]; then
  echo "Killing existing tmux session: $SESSION"
  tmux kill-session -t $SESSION
fi

# Create new tmux session and window
tmux new-session -d -s $SESSION -n main

# Pane 1: Launch rosbridge_websocket
tmux send-keys -t $SESSION "source $INSTALL_DIR/setup.bash" C-m
tmux send-keys -t $SESSION 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' C-m

# Split window horizontally and run OpenMCT
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $OPENMCT_DIR && npm start" C-m


# Attach to the session
tmux attach-session -t $SESSION
