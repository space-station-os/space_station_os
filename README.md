# **Space Station OS – Setup & Demo Guide**

<p align="center">
  <img src="https://raw.githubusercontent.com/space-station-os/space_station_os/main/assets/logo/SSOSlogo.jpg" alt="SSOS Logo" width="594"/>
</p>

[![ROS 2 Humble CI](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml)

---

## 🚀 Quick Start with Docker (Recommended)

If you prefer not to build everything locally, use our **prebuilt Docker image** to get up and running instantly.

### 1. Pull the image

```bash
docker pull ghcr.io/space-station-os/space_station_os:latest
```

> No need to install ROS 2 manually. Docker must be installed and running.

### 2. Run the container

```bash
docker run -it --rm ghcr.io/space-station-os/space_station_os:latest
```

You'll be dropped into a ready-to-use environment with ROS 2 Humble and all Space Station OS packages pre-built.

---

## 🔧 Local Installation (Build from Source)

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
source install/setup.bash
```

> Always source the workspace before running ROS 2 commands:
>
> ```bash
> source ~/ssos_ws/install/setup.bash
> ```

---

## 🛰️ Running Demo 1: ISS Incident Simulation

You need **3 terminals** for this demo.

### Terminal 1 – Run a Scenario

Choose one:

```bash
ros2 run space_station_gnc demo1a_nauka_incident_estimate     # Real-world based
ros2 run space_station_gnc demo1b_crisis_mainengine           # High-severity test
ros2 run space_station_gnc demo1c_small_incident              # CMG logic test
```

### Terminal 2 – Launch GNC Core

```bash
ros2 launch space_station_gnc gnc_core.launch.py
```

### Terminal 3 – Launch RViz

```bash
ros2 launch space_station_gnc gnc_rviz.launch.py
```

Set `Fixed Frame` to `world`, add `RobotModel`, and choose `/robot_description`.

---

## 🧩 Environmental Control and Life Support System (ECLSS)

ECLSS simulates the life support systems needed to maintain habitable space environments.

### Subsystems Implemented:

* **ARS** – CO₂ and contaminant removal
  🔗 [Air Revitalization System Docs](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/ars_systems/README.md)

* **ORS** – Oxygen generation from water
  🔗 [Oxygen Recovery System Docs](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/ors_systems/README.md)

* **WRS** – Water recycling & purification
  🔗 [Water Recovery System Docs](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/wrs_systems/README.md)

### Launch All Systems

```bash
ros2 launch space_station_eclss eclss.launch.py
```

---

## 🛰️ Space Station Gazebo Simulation

Visualize the Haven-2 model in Gazebo:

```bash
ros2 launch space_station_description gazebo.launch.py
```

### Teleoperation

```bash
ros2 run space_station_description mux
ros2 run space_station_description teleop
```

---

## 🐳 Building the Docker Image Locally (Optional)

If the prebuilt image doesn’t work, you can build it manually.

### 1. Clone the repo

```bash
git clone https://github.com/space-station-os/space_station_os.git
cd space_station_os
```

### 2. Build the image

```bash
docker build -t space_station_os:latest .
```

### 3. Run the container

```bash
docker run -it --rm space_station_os:latest
```

---



##  Contributing

See the project backlog:
[Space Station OS – Project Board](https://github.com/orgs/space-station-os/projects/2/views/1)
