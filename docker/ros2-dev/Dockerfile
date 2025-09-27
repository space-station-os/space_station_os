# Base ROS 2 image
FROM osrf/ros:jazzy-desktop

# Maintainer
LABEL maintainer="Space Station OS <spacestationos@spacedata.co.jp>"

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV QT_X11_NO_MITSHM=1
ENV ROS_DOMAIN_ID=23
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_SOFTWARE=1 

# Install system packages + Node.js + npm
RUN apt update && apt install -y \
    git \
    curl \
    wget \
    tmux \
    vim \
    nano \
    htop \
    lsb-release \
    gnupg \
    ranger \
    python3-pip \
    python3-rosdep \
    build-essential \
    libopencv-dev \
    python3-opencv \
    python3-pyqt5 \
    libgl1 \
    libglib2.0-0 \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-actuator-msgs \
    ros-jazzy-rosbridge-server \
    ros-jazzy-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-* \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-behaviortree-cpp-v3 \
    ros-dev-tools \
 && curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
 && apt install -y nodejs \
 && npm install -g npm@11.5.1 \
 && rm -rf /var/lib/apt/lists/*

# Install CasADi
RUN pip3 install --break-system-packages casadi

# GZ + URDF resources
ENV GZ_VERSION=harmonic
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz/harmonic:/root/ssos_ws/src/space_station_description/urdf:/root/ssos_ws/src/space_station_description/meshes

# ROS workspace setup
RUN sudo rosdep init || true && rosdep update

WORKDIR /root/ssos_ws
RUN mkdir -p src

# Copy workspace source code
COPY . src/space_station_os

# Install ROS dependencies
RUN bash -c "source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
RUN rosdep resolve gz-harmonic || true

# Build workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Copy and configure entrypoint
COPY entry-point.sh /root/entry-point.sh
RUN chmod +x /root/entry-point.sh
