# Space Station OS

https://github.com/user-attachments/assets/5b82c516-075d-44ac-b440-fb73bedc1e91

[![CI](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ci.yml)
[![Pixi CI](https://github.com/space-station-os/space_station_os/actions/workflows/pixi.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/pixi.yml)

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

| Package | What it is |
|---------|------------|
| `space_station_eclss` | Environmental Control and Life Support System |
| `space_station_eps` | Electrical Power System |
| `space_station_gnc` | Guidance, Navigation and Control |
| `space_station_thermal_control` | Active Thermal Control System |
| `space_station_communication` | Communications |
| `space_station_interfaces` | Shared ROS 2 messages, services, and actions |
| `space_station_mission_control` | OpenMCT dashboards and the OpenMCT-ROS bridge |

---

## Choose how to use SSOS

| Goal | Recommended path |
|------|------------------|
| Try SSOS on Ubuntu without using the terminal after installation | Tier-1 Desktop App |
| Run SSOS from the terminal without installing system ROS 2 | Pixi |
| Develop SSOS in a reproducible environment | Pixi |
| Develop and integrate against a native ROS 2 installation | Ubuntu 24.04 + ROS 2 Jazzy |
| Work with the containerized environment | Docker (advanced) |

Pixi is the recommended reproducible environment for most SSOS development.
Native ROS 2 Jazzy is supported for system integration and development against
a system ROS installation.

---

## Try SSOS as a Desktop App

The Tier-1 Desktop App is supported on Ubuntu 24.04 Desktop. It creates a
Space Station OS shortcut that launches the full mission-control stack through
the repository's Pixi environment.

Starting from a clean Ubuntu 24.04 Desktop installation:

```bash
sudo apt update
sudo apt install -y curl git zenity

curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.pixi/bin:$PATH"

git clone https://github.com/space-station-os/space_station_os.git
cd space_station_os

bash desktop/install.sh
```

The installer uses the committed `pixi.lock`, builds SSOS, and places a
**Space Station OS** icon on your Desktop.

Double-click the icon to launch SSOS. On first launch, if GNOME displays an
untrusted-launcher warning, right-click the icon and choose **Allow Launching**.

To uninstall the shortcut:

```bash
bash desktop/uninstall.sh
```

The repository and Pixi environment are not removed.

See [docs/DESKTOP_APP.md](docs/DESKTOP_APP.md) for details and troubleshooting.

---

## Run SSOS from the Terminal with Pixi

Pixi provides ROS 2 Jazzy through RoboStack and does not require a system ROS 2
installation, `rosdep`, or a ROS-specific `apt` setup.

Starting from a clean Ubuntu installation:

```bash
sudo apt update
sudo apt install -y curl git

curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.pixi/bin:$PATH"

git clone https://github.com/space-station-os/space_station_os.git
cd space_station_os

pixi install --locked
pixi run build
pixi run test
pixi run station
```

`pixi install --locked` installs the environment recorded in the committed
`pixi.lock` and prevents normal setup from silently regenerating dependency
versions.

See [docs/PIXI.md](docs/PIXI.md) for the environment structure and dependency
update procedure.

---

## Develop SSOS

### Recommended: Pixi

Pixi is the recommended development environment when you want a reproducible
SSOS toolchain without modifying the host ROS installation.

After cloning your working repository:

```bash
cd space_station_os

pixi install --locked
pixi run build
pixi run test
pixi run station
```

Common Pixi tasks:

| Task | Purpose |
|------|---------|
| `pixi run build` | Build the current SSOS package set with colcon |
| `pixi run test` | Run the current Pixi validation test set |
| `pixi run station` | Build and launch the full SSOS stack |
| `pixi run gui` | Launch the mission-control GUI |
| `pixi run clean` | Remove `build/`, `install/`, and `log/` |

Do not regenerate `pixi.lock` during normal setup. Contributors intentionally
changing dependencies should follow the procedure in
[docs/PIXI.md](docs/PIXI.md).

### Native ROS 2 Jazzy

Use the native path when developing or integrating SSOS against a system ROS 2
installation.

Prerequisites:

- Ubuntu 24.04
- ROS 2 Jazzy Desktop
- ROS development tools

Follow the official ROS 2 Jazzy installation guide to install both ROS 2 Jazzy
Desktop and the ROS development tools before continuing below.

Install ROS 2 Jazzy using the official documentation:

[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Create a workspace and clone SSOS:

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src

git clone https://github.com/space-station-os/space_station_os.git
cd ~/ssos_ws
```

Install dependencies:

```bash
source /opt/ros/jazzy/setup.bash

sudo rosdep init   # Skip if rosdep has already been initialized.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build and test:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --verbose
```

Launch the full station:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ssos_ws/install/setup.bash

ros2 launch space_station space_station.launch.py
```

Source both ROS 2 Jazzy and the workspace overlay in each new terminal before
running SSOS commands.

See [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) for the complete contributor
workflow.

---

## Docker (Advanced)

Docker is maintained as an advanced environment and is not part of the
Tier-1 v0.9 setup path. Full Docker validation is handled separately.

The current container workflow uses the published GHCR image and X11 GUI
forwarding.

Pull the image:

```bash
docker pull ghcr.io/space-station-os/space_station_os:latest
```

Allow the local container to reach the X server:

```bash
xhost +local:root
```

Run the container:

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  ghcr.io/space-station-os/space_station_os:latest
```

Docker behavior and GUI compatibility should be validated separately from the
Tier-1 Desktop App and Pixi setup paths.

---

## Running Individual Systems

With a native ROS 2 workspace sourced, the full current architecture can be
launched with:

```bash
ros2 launch space_station space_station.launch.py
```

Legacy subsystem launch files are also available:

```bash
ros2 launch space_station eclss.launch.py
ros2 launch space_station eps.launch.py
ros2 launch space_station gnc.launch.py
ros2 launch space_station thermals.launch.py
```

### OpenMCT dashboard

To run the OpenMCT bridge:

```bash
./open_mct-bridge.sh
```

Then open `http://localhost:9097`.

---

## Documentation

- [docs/PIXI.md](docs/PIXI.md) - reproducible Pixi build, run, and dependency management
- [docs/DESKTOP_APP.md](docs/DESKTOP_APP.md) - Tier-1 Desktop App installation and updates
- [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) - contributor setup and development workflow
- [Project wiki](https://github.com/space-station-os/space_station_os/wiki)

---

## Contributing

See [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md) and the project backlog:

[Space Station OS - Project Board](https://github.com/orgs/space-station-os/projects/2/views/1)
