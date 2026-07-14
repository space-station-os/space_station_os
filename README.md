# **Space Station OS – Setup & Demo Guide**



https://github.com/user-attachments/assets/5b82c516-075d-44ac-b440-fb73bedc1e91



[![CI](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml)

---

##  Quick Start with pixi (reproducible local build)

[pixi](https://pixi.sh) gives you a pinned ROS 2 Jazzy environment (via
[RoboStack](https://robostack.github.io)) with no system ROS, `apt`, or `rosdep`.
From a clean checkout:

```bash
pixi install        # resolve + fetch ROS 2 Jazzy + build tools + GUI deps
pixi run build      # colcon build space_station, ssos_core, ssos_sim, ssos_eclss
pixi run station    # launch the full stack (GUI + core + sim + eclss)
```

Other tasks: `pixi run gui`, `pixi run test`, `pixi run clean`. Scope and details
in [docs/PIXI.md](docs/PIXI.md). Linux (`linux-64`) is supported first.

---

##  Install as a Desktop App (Ubuntu)

Prefer clicking an icon over the terminal? Install a desktop shortcut that
launches the full mission-control stack. Requires [pixi](https://pixi.sh).

```bash
cd space_station_os
bash desktop/install.sh
```

A progress popup runs `pixi install` + `pixi run build`, then places a
**Space Station OS** icon on your Desktop. Double-click it to launch (on the
first launch, if GNOME warns, right-click the icon and choose "Allow Launching").
Uninstall with `bash desktop/uninstall.sh`. Full details in
[docs/DESKTOP_APP.md](docs/DESKTOP_APP.md).

---

##  Quick Start with Docker (with GUI support)

If you prefer not to build everything locally, use our **prebuilt Docker image** to get up and running instantly — including GUI support for the astronaut simulation.

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

---

##  Local Installation (Build from Source)

Use this method if you want to modify the source code or don't want to use Docker.

### Prerequisites

* **OS:** Ubuntu 22.04
* **ROS 2:** Humble (Desktop)
  → [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src
```

### 2. Clone the super-repository with submodules

```bash
git clone https://github.com/space-station-os/space_station_os.git
cd space_station_os
```

### 3. Build the workspace

Go back to the workspace root and build everything:

```bash
cd ~/ssos_ws
colcon build --symlink-install
sudo rosdep init
rosdep update 
rosdep install --from-paths src --ignore-src -r -y
source install/setup.bash
```

> Always source the workspace before running ROS 2 commands:
>
> ```bash
> source ~/ssos_ws/install/setup.bash
> ```

---
# TO RUN THE DEMOS

To quickly run the entire space station 

```bash
ros2 launch space_station space_station.launch.py
```

To run OpenMCT (open http://localhost:9097)

```bash
./open_mct-bridge.sh
```

([Check out our wiki](https://github.com/space-station-os/space_station_os/wiki))
##  Contributing

See the project backlog:
[Space Station OS – Project Board](https://github.com/orgs/space-station-os/projects/2/views/1)
