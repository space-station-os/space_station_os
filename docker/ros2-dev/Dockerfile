# Base ROS 2 image
FROM osrf/ros:humble-desktop

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV QT_X11_NO_MITSHM=1
ENV ROS_DOMAIN_ID=23
# ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ENV ROS_DISCOVERY_SERVER=10.0.0.109:11811
# ENV ROS_SUPER_CLIENT=1
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_SOFTWARE=1 
# ENV QT_DEBUG_PLUGINS=1

# Install system packages
RUN apt update && apt install -y \
    git \
    curl \
    wget \
    tmux \
    ranger \
    python3-pip \
    python3-rosdep \
    build-essential \
    libopencv-dev \
    python3-opencv \
    python3-pyqt5 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-actuator-msgs \
    ros-humble-rosbridge-server \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install casadi

# Initialize rosdep
RUN sudo rosdep init || true && rosdep update

# Create workspace
WORKDIR /root/ssos_ws
RUN mkdir -p src

# Copy your monorepo into the workspace
COPY . src/space_station_os

# Install dependencies
RUN bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Copy and configure entrypoint to start Discovery Server and shell
COPY entry-point.sh /root/entry-point.sh
RUN chmod +x /root/entry-point.sh

