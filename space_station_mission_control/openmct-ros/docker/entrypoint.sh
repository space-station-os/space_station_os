#!/bin/bash

# Source the ROS setup script
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd /app/ros2_ws

# Source the workspace
source install/local_setup.bash

# Execute the passed command
exec "$@"