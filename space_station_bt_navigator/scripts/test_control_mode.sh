#!/bin/bash

echo "=== GNC Control Mode Switching Test ==="

source /opt/ros/humble/setup.bash
cd /home/snkt/ssoc_ws

echo "Building workspace..."
colcon build --packages-select space_station_gnc space_station_bt_nodes space_station_bt_navigator

if [ $? -ne 0 ]; then
    echo "Build failed! Exiting."
    exit 1
fi

echo "Build successful! Sourcing workspace..."
source install/setup.bash

echo "Starting control mode switching test..."

echo "Launching control mode test..."
ros2 launch space_station_bt_navigator control_mode_test.launch.py &
LAUNCH_PID=$!

sleep 5

echo "Checking if nodes are running..."
ros2 node list

echo "Checking control mode service..."
ros2 service list | grep gnc

echo "Checking control mode topics..."
ros2 topic list | grep gnc

echo "Current control mode:"
ros2 topic echo /gnc/current_control_mode --once

echo "Control mode status:"
ros2 topic echo /gnc/control_mode_status --once

echo "Test completed. Press Ctrl+C to stop the launch."
wait $LAUNCH_PID
