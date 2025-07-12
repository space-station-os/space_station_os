
# Air Revitalization System (ARS) 

### Overview
The **Air Revitalization System (ARS)** is a ROS 2-based environmental life support simulation for closed-loop air processing in space habitats. It models an advanced 5-stage system inspired by the ISS CDRA and Vozdukh architecture, with realistic adsorption, humidification, and desorption dynamics driven by ROS 2 services and topic streams.

---

### System Architecture

The ARS system is composed of five main ROS 2 nodes:

| Node                | Function                                   |
|---------------------|--------------------------------------------|
| `air_collector_tank` | Collects cabin air, monitors contaminants |
| `desiccant_tank_1`   | Removes moisture and contaminants         |
| `adsorbent_tank_1`   | Adsorbs CO₂ using Langmuir kinetics       |
| `adsorbent_tank_2`   | Desorbs CO₂ and publishes to storage      |
| `desiccant_tank_2`   | Rehumidifies scrubbed air before recirculation |

---

### Data Flow Summary

```text
Air Collector → Desiccant Bed 1 → Adsorbent Bed 1 → Adsorbent Bed 2 → CO₂ Storage
                                         ↓
                             Desiccant Bed 2 (humidifies)
```

---


### Launching the System

Make sure you've built the workspace:

```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

Launch all components:

```bash
ros2 launch demo_nova_sanctum ars_systems_v4.launch.py
```

Or run each manually:

```bash
ros2 run demo_nova_sanctum air_collector_tank
ros2 run demo_nova_sanctum desiccant_tank_1
ros2 run demo_nova_sanctum adsorbent_tank_1
ros2 run demo_nova_sanctum adsorbent_tank_2
ros2 run demo_nova_sanctum desiccant_tank_2
```


## REFERENCE

![image](https://github.com/user-attachments/assets/36b7c3ba-2394-422c-a920-b9a2a564ca5f)


