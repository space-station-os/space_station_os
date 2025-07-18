#!/bin/bash

# Start Fast DDS discovery server in background
tmux new -s discovery -d "fastdds discovery --server-id 0"
sleep 2  # give time to init

