#!/bin/bash

echo "=== CMG Unloading Behavior Tree Test ==="

source /opt/ros/humble/setup.bash
cd /home/snkt/ssoc_ws

echo "Building workspace..."
colcon build --packages-select space_station_bt_nodes space_station_bt_navigator

if [ $? -ne 0 ]; then
    echo "Build failed! Exiting."
    exit 1
fi

echo "Build successful! Sourcing workspace..."
source install/setup.bash

echo "Starting CMG unloading test..."

echo "Launching CMG unloading behavior tree test..."
ros2 launch space_station_bt_navigator cmg_unloading_test.launch.py &
LAUNCH_PID=$!

sleep 5

echo "Checking if nodes are running..."
ros2 node list

echo "Checking CMG topics..."
ros2 topic list | grep cmg

echo "Monitoring CMG status for 30 seconds..."
timeout 30s ros2 topic echo /gnc/cmg_status

echo "Triggering CMG unloading..."
ros2 topic pub /gnc/unload_cmg std_msgs/msg/Empty "{}" --once

echo "Monitoring unloading progress for 60 seconds..."
timeout 60s ros2 topic echo /gnc/cmg_h

echo "Final CMG status:"
ros2 topic echo /gnc/cmg_status --once

echo "Test completed! Cleaning up..."
kill $LAUNCH_PID

echo "=== Test finished ==="
