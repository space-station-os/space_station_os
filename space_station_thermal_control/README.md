# Space Station Thermal Control – Demo Guide

This demo simulates the active/passive thermal control system of a space station module, including internal coolant loop, radiators, solar panel heat input, and a heat conduction graph over the station surface.

---

## How to Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## How to Run the Thermal Control Demo

Use **3 terminals** as follows:

---

### Terminal 1 – Solar Absorptivity + Sun Vector Node

```bash
ros2 launch space_station_thermal_control solar_absorpitivity.launch.py
```

- Launches `sun_vector` and `array_absorptivity` nodes
- Uses `solar_array.yaml` for panel absorptivity parameters
- Outputs panel-level alpha and area setup to log

---

### Terminal 2 – Thermal Control System Core

```bash
ros2 launch space_station_thermal_control thermals.launch.py
```

- Launches:
  - `internal_coolant`: coolant system simulating heating/cooling
  - `external_loop`: ammonia-based external loop
  - `radiator`: exports heat to space
  - `thermal_nodes`: heat graph of the station body
  - `robot_state_publisher` with built-in URDF

Expected log:
```bash
[INFO] [thermal_network]: Thermal graph initialized with 29 nodes and 29 links.
[INFO] [thermal_network]: Avg temperature of node = 314.7
```

---

### Terminal 3 – RViz Visualization

```bash
ros2 run rviz2 rviz2
```

Then manually configure:
- Set `Fixed Frame` to `base_link`
- Add `RobotModel` display
- Confirm `robot_description` is published (topic is OK)

If the model does not appear, try:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link
```

---

## Temperature Output

The `thermal_nodes` node will output average temperature rise in the station over time.

## Observing Thermal Simulation Topics

To monitor the behavior of the thermal control system in real-time, you can echo the following ROS 2 topics:

### Node Temperatures (evolving over time)

```bash
ros2 topic echo /thermal/nodes/state
```
This topic publishes the current temperature of each thermal node in the network. You should observe gradual changes based on heat absorption and coolant dynamics.

#### Plot Node Temperatures 

- Step1. record the topic to save it to a file (e.g., node_temp.log)

```bash
timeout 30 ros2 topic echo /thermal/nodes/state > node_temp.log
```

- Step2. record the topic to save it to a file (e.g., node_temp.log)

```bash
python3 ~/ros2_ws/src/space_station_os/space_station_thermal_control/analysis/plot_node_temp.py -f (filename to saved in Step1) -n (name of node) -t (time duration in sec)
```

### Solar Heat Input

```bash
ros2 topic echo /thermal/solar_heat
```
This topic shows the solar heat input for each solar array panel, based on their absorptivity and sun vector angle.
This feature is under development.

### Solar Heat Input

```bash
ros2 topic echo /thermal/links/flux
```
This topic is designed to represent conductive/radiative heat transfer between connected components. At present, it may publish heat_flow = 0.0 as this feature is under development.


---

## Transforms

Use these tools to inspect:

```bash
ros2 topic echo /robot_description
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

- `robot_state_publisher` must not be launched multiple times.
- Remove or rename `~/.rviz2/default.rviz` if RViz has invalid config.
- Ensure `robot_description` topic is active.

---

## Launch Files

- `solar_absorpitivity.launch.py` – sun vector & solar panel heat input
- `thermals.launch.py` – coolant, radiator, URDF, and thermal graph

---

## TODO List

This list is as of 2025-07-13:

- [done]  Launch scripts confirmed: solar_absorpitivity.launch.py, thermals.launch.py
- [done]  All thermal control nodes are executing and publishing
- [todo]  Visual confirmation in RViz is currently unstable or missing
- [todo]  /thermal/nodes/state publishing confirmed; temperature increasing observed
- [todo]  /thermal/solar_heat publishes heat inputs — not yet linked to internal_power
- [todo]  heat_flow field in /thermal/links/flux remains at 0.0 — needs implementation
- [todo]  Cooling service exists but is stubbed ([FILL] Water service not available.)
- [todo]  Parameter mapping from URDF or YAML not fully validated
- [todo]  Time resolution (dt = 0.5) and RK4 method need validation for physical plausibility
- [todo]  Visual graphing (e.g., rqt_plot) not yet tested

---


