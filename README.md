# **Space Station OS – Setup & Demo Guide**



https://github.com/user-attachments/assets/5b82c516-075d-44ac-b440-fb73bedc1e91



[![ROS 2 Humble CI](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml)

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

([Check out our wiki](https://github.com/space-station-os/space_station_os/wiki))

## Behavior Tree
The behavior tree subsystem is composed of three packages:

- `space_station_bt` implements the core `BehaviorTreeEngine` responsible for loading plugins and executing trees.
- `space_station_bt_nodes` contains the plugin libraries for tree nodes (actions, conditions, decorators, controls).
- `space_station_bt_navigator` instantiates the engine inside a ROS 2 node to run behavior trees.

Typical workflow:
1. The navigator loads the node plugin library and constructs a tree from XML.
2. The engine executes the tree, invoking plugins as it traverses nodes.
3. Plugins implement spacecraft behaviors which can be expanded over time.

An example tree demonstrating a failsafe that unloads the CMGs is provided in
`space_station_bt_navigator/failsafe_tree.xml`.

For an in-depth walkthrough of the BT engine, navigator and node plugins,
refer to **behaviourTree.md** in this repository.

### Running the Behavior Tree demo

After building the workspace and sourcing the `install` setup script, start the
demo which includes the lifecycle manager:

```bash
ros2 run space_station_bt_navigator bt_demo
```

The `LifecycleManager` configures and activates the `BtNavigator` before the
tree begins execution and will shut it down when the demo finishes. `BtNavigator`
is itself a lifecycle node so you can drive its transitions with the standard
`ros2 lifecycle` command line tool.

### Controlling lifecycle nodes manually

You can also drive lifecycle transitions yourself. For example:

```bash
# Configure and activate the navigator
ros2 lifecycle set /bt_navigator configure
ros2 lifecycle set /bt_navigator activate

# Deactivate and cleanup when done
ros2 lifecycle set /bt_navigator deactivate
ros2 lifecycle set /bt_navigator cleanup
```

### Additional packages

The behavior tree stack relies on three helper packages that mirror the
Navigation2 architecture:

- `space_station_utils` provides small header-only helpers such as the
  `PluginLoader` wrapper and parameter utilities.
- `space_station_core` defines abstract plugin interfaces that subsystems can
  implement.
- `space_station_lifecycle_manager` offers a node to configure and activate a
  list of lifecycle-enabled nodes before execution.

---

##  Contributing

See the project backlog:
[Space Station OS – Project Board](https://github.com/orgs/space-station-os/projects/2/views/1)
