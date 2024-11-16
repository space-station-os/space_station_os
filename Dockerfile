FROM osrf/ros:humble-desktop

RUN apt update && apt install -y xterm

WORKDIR /root/space_station_os

COPY src ./src
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"
