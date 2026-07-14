# Space Station OS

https://github.com/user-attachments/assets/5b82c516-075d-44ac-b440-fb73bedc1e91

[![CI](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml)

Space Station OS (SSOS) is an open-source simulation of a crewed space station and
its subsystems, built on ROS 2. It models the closed-loop physics of life support,
power, thermal, GNC, and communications, coordinated by a shared core and a
simulation controller, and visualized through a mission-control GUI.

The goal is a realistic, extensible platform for people building, researching, or
learning about space stations and their subsystems.

---

## Repository layout

| Package | What it is |
|---------|------------|
| `ssos_core` | System manager: global state model, subsystem registration, fault aggregation |
| `ssos_sim` | Simulation controller: world model, sim clock, scenario loading, fault injection |
| `ssos_eclss` | High-fidelity ECLSS physics (ARS / WRS / OGS, crew, closed loop) |
| `space_station` | Mission-control GUI and the top-level launch files |

### Legacy Systems
| `space_station_eclss` | Environmental Control and Life Support System |
| `space_station_eps` | Electrical Power System |
| `space_station_gnc` | Guidance, Navigation and Control |
| `space_station_thermal_control` | Active Thermal Control System |
| `space_station_communication` | Communications |
| `space_station_interfaces` | Shared ROS 2 messages, services, and actions |
| `space_station_mission_control` | OpenMCT dashboards and the OpenMCT-ROS bridge |

---

## Quick Start with pixi (reproducible local build)

[pixi](https://pixi.sh) gives you a pinned ROS 2 Jazzy environment (via
[RoboStack](https://robostack.github.io)) with no system ROS, `apt`, or `rosdep`.
From a clean checkout:

```bash
pixi install        # resolve + fetch ROS 2 Jazzy + build tools + GUI deps
pixi run build      # colcon build space_station, ssos_core, ssos_sim, ssos_eclss
pixi run station    # launch the full stack (GUI + core + sim + eclss)
```

---

## Install as a Desktop App (Ubuntu)

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

## Quick Start with Docker (with GUI support)

If you prefer not to build locally, use the prebuilt Docker image, including GUI
support for the simulation.

### 1. Pull the image

```bash
docker pull ghcr.io/space-station-os/space_station_os:latest
```

> Docker must be installed and running. No need to install ROS 2 or dependencies manually.

### 2. Allow GUI access

Allow local Docker containers to reach your X server:

```bash
xhost +local:root
```

### 3. Run the container

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

## Build from source

Use this if you want to modify the code and are not using pixi or Docker.

### Prerequisites

* **OS:** Ubuntu 24.04
* **ROS 2:** Jazzy (Desktop)
  -> [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 1. Create a workspace and clone

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src
git clone https://github.com/space-station-os/space_station_os.git
```

### 2. Install dependencies and build

```bash
cd ~/ssos_ws
sudo rosdep init      # first time only; skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> Source the workspace in every new terminal before running ROS 2 commands:
>
> ```bash
> source ~/ssos_ws/install/setup.bash
> ```

---

## Running the simulation

Launch the full space station (New systems following SSOS 2026 architecture):

```bash
ros2 launch space_station space_station.launch.py
```

Or launch a single subsystem (Legacy systems):

```bash
ros2 launch space_station eclss.launch.py
ros2 launch space_station eps.launch.py
ros2 launch space_station gnc.launch.py
ros2 launch space_station thermals.launch.py
```

### OpenMCT dashboard

To run the OpenMCT bridge (then open http://localhost:9097):

```bash
./open_mct-bridge.sh
```

---

## Documentation

* [docs/PIXI.md](docs/PIXI.md) - reproducible pixi build and run
* [docs/DESKTOP_APP.md](docs/DESKTOP_APP.md) - desktop app install and updates
* [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) - how to contribute
* [Project wiki](https://github.com/space-station-os/space_station_os/wiki)

---

## Contributing

See [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) and the project backlog:
[Space Station OS - Project Board](https://github.com/orgs/space-station-os/projects/2/views/1)
