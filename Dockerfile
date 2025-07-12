FROM osrf/ros:humble-desktop

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies
RUN apt update && apt install -y \
    git \
    curl \
    wget \
    tmux \
    ranger \
    python3-pip \
    python3-rosdep \
    build-essential \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-actuator-msgs \
    ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependency: CasADi
RUN pip3 install casadi

# Initialize rosdep
RUN rosdep init || true && rosdep update        

# Create clean ROS 2 workspace layout
WORKDIR /root/ssos_ws
RUN mkdir -p src

# Copy the entire monorepo into src/
COPY . src/space_station_os

# Set working directory inside workspace
WORKDIR /root/ssos_ws

# Install ROS 2 package dependencies 
RUN bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source the workspace on container startup
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec bash"]
