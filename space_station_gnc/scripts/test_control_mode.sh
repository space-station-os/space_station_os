#!/bin/bash

# Test script for GNC control mode switching

echo "=== GNC Control Mode Switching Test ==="

# Set up the workspace
source /opt/ros/humble/setup.bash
cd /home/snkt/ssoc_ws

echo "Building GNC package..."
colcon build --packages-select space_station_gnc

if [ $? -ne 0 ]; then
    echo "Build failed! Exiting."
    exit 1
fi

echo "Build successful! Sourcing workspace..."
source install/setup.bash

echo "Starting control mode test..."

# Launch the control mode test
echo "Launching control mode test..."
ros2 launch space_station_gnc control_mode_test.launch.py &
LAUNCH_PID=$!

# Wait for launch to complete
sleep 5

echo "Checking if nodes are running..."
ros2 node list

echo "Checking control mode topics..."
ros2 topic list | grep gnc

echo "Current control mode:"
ros2 topic echo /gnc/current_control_mode --once

echo "Control mode status:"
ros2 topic echo /gnc/control_mode_status --once

echo "Testing control mode switching..."

# Test switching to thruster mode
echo "Switching to thruster mode..."
ros2 topic pub /gnc/set_control_mode std_msgs/msg/Int8 "data: 1" --once

sleep 2

echo "Current control mode after switch:"
ros2 topic echo /gnc/current_control_mode --once

echo "Control mode status after switch:"
ros2 topic echo /gnc/control_mode_status --once

# Test switching back to CMG mode
echo "Switching back to CMG mode..."
ros2 topic pub /gnc/set_control_mode std_msgs/msg/Int8 "data: 0" --once

sleep 2

echo "Current control mode after switch back:"
ros2 topic echo /gnc/current_control_mode --once

echo "Control mode status after switch back:"
ros2 topic echo /gnc/control_mode_status --once

echo "Test completed. Press Ctrl+C to stop the launch."
wait $LAUNCH_PID
