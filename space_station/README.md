
# SPACE STATION OS – GUI Testing with GPT-oss

This repository demonstrates the **Space Station OS** GUI integrated with the **gpt-oss-20b** open-source model via NVIDIA’s Integrate API.  
The AI agent (`space_station/ai_agent.py`) reads telemetry from ROS 2 topics and answers astronaut questions using **gpt-oss-20b**.

---

## 1. Docker Setup

Make sure you have Docker installed (with GUI/X11 support).

### Pull the Image
```bash
docker pull ghcr.io/space-station-os/space_station_os:latest
````

> Docker must be installed and running. No need to install ROS 2 or dependencies manually.

### Allow GUI Access

```bash
xhost +local:root
```

### Run the Container with GUI Support

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  ghcr.io/space-station-os/space_station_os:latest
```

---

## 2. Clone and Build the GUI Branch

> NOTE: Remove any existing repo clone and use the `feature/100-gui` branch.

```bash
mkdir -p test_ws/src
cd test_ws/src
git clone -b feature/100-gui https://github.com/space-station-os/space_station_os.git
cd ..
colcon build --symlink-install
```

---

## 3. Run the GUI

```bash
ros2 launch space_station space_station.launch.py
```

---

## 4. GPT-oss-20b Integration

The AI agent in `space_station/ai_agent.py` is configured to use the **gpt-oss-20b** model by default.



## License

This repository is distributed under an open-source license (Apache-2.0 recommended).

