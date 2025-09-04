# === Base ROS 2 Jazzy Desktop image ===
FROM osrf/ros:jazzy-desktop

LABEL maintainer="Space Station OS <spacestationos@spacedata.co.jp>"

# --- Environment Variables ---
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV QT_X11_NO_MITSHM=1
ENV ROS_DOMAIN_ID=23
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MPLBACKEND=Qt5Agg  
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# --- System packages ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl wget tmux vim nano htop lsb-release gnupg ranger \
    python3-pip python3-rosdep build-essential \
    libopencv-dev python3-opencv \
    python3-pyqt5 python3-pyqt5.qtwebsockets \
    python3-matplotlib libgl1 libglib2.0-0 \
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
    ros-dev-tools \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

# --- Node.js (optional, for web dashboard etc.) ---
RUN curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
 && apt-get install -y nodejs \
 && npm install -g npm@11.5.1

# --- Python packages (via pip) ---
RUN pip3 install --no-cache-dir --break-system-packages \
    casadi \
    python-dotenv \
    openai

# --- GZ Sim Resources ---
ENV GZ_VERSION=harmonic
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz/harmonic:/root/ssos_ws/src/space_station_description/urdf:/root/ssos_ws/src/space_station_description/meshes

# --- ROS Workspace Setup ---
RUN rosdep init || true && rosdep update

WORKDIR /root/ssos_ws
RUN mkdir -p src

# Copy workspace source code
COPY . src/space_station_os

# Install ROS dependencies
RUN bash -lc "source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Just in case (optional): suppress failures resolving specific GZ variants
RUN rosdep resolve gz-harmonic || true

# Build workspace
RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Entrypoint
COPY entry-point.sh /root/entry-point.sh
RUN chmod +x /root/entry-point.sh


