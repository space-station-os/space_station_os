## SPACE STATION OS- GUI TESTING with GPT-oss

Make sure you have docker support with ROS2 installed or use the below instructions

### 1. Pull the image

```bash
docker pull ghcr.io/space-station-os/space_station_os:latest
```

> Docker must be installed and running. No need to install ROS 2 or dependencies manually.

---

### 2. Allow GUI access

Before running the container, allow local Docker containers to access your X server:

```bash
xhost +local:root
```

---

### 3. Run the container with GUI support

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  ghcr.io/space-station-os/space_station_os:latest
```

> NOTE: Remove the existing repo and install the ``feature/100-gui`` branch 

``bash
mkdir -p test_ws/src
cd src
git clone -b feature/100-gui https://github.com/space-station-os/space_station_os.git
cd ..
colcon build --symlink-install
``

## TO RUN THE GUI 

```bash
ros2 launch space_station space_station.launch.py 
```

