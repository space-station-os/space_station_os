# Space Station Thermal Control Subsystem

This package simulates the **Active Thermal Control System (ATCS)** of a space station. It models how heat is generated, accumulated in station nodes, transferred to coolant loops, and finally vented into space through radiators.

The system is built on ROS 2 using **actions, services, and topic publishers/subscribers** to mimic the real flow of thermal energy inside a space station.

---

## 🌍 How It Works

The thermal control simulation runs as a **closed loop**, with the following steps:

1. **Solar Heating** 

   * The `sun_vector` node publishes the position of the Sun.
   * The `solar_heat_node` converts this into **heat input** for the station’s thermal nodes.

2. **Thermal Nodes Heat Up** 

   * The `thermals_solver` node integrates heat flow across all nodes and links (defined in a YAML config).
   * It publishes each node’s temperature on `/thermal/nodes/state`.
   * If average temperature exceeds a threshold, it triggers cooling.

3. **Coolant Loop Activated** 
4. 
   * `thermals_solver` sends an **action goal** to `/coolant_heat_transfer` handled by `cooling.cpp` (`CoolantActionServer`).
   * The coolant absorbs heat based on:

     $$
     Q = m \cdot C_p \cdot \Delta T
     $$
   * Internal water loop temperature is updated and published on `/tcs/internal_loop_heat`.

5. **Heat Transferred to Ammonia Loop** 

   * The coolant loop transfers heat to the ammonia loop with efficiency losses.
   * This is published on `/tcs/external_loop_a/status`.

6. **Radiator Venting** 

   * If ammonia heat exceeds a threshold, the `CoolantActionServer` calls the `/tcs/radiator_a/vent_heat` service.
   * The `radiators.cpp` (`RadiatorController`) handles this by commanding solar joints (via `/solar_controller/commands`) to rotate and dump heat to space.

7. **Water Recycling** 

   * After several cooling cycles, the coolant server automatically:

     * Requests fresh water (`/wrs/product_water_request`).
     * Dumps used water into `/grey_water`.

8. **Diagnostics** 

   * Throughout the process, nodes publish to `/thermals/diagnostics` to flag overheating, failed cooling, or service issues.

---

##  Nodes Overview

* **`thermals_solver`**
  Core simulator: integrates node temps, detects overheat, triggers cooling.

* **`cooling` (`CoolantActionServer`)**
  Action server for cooling: absorbs heat, transfers to ammonia, vents via radiator.

* **`radiators` (`RadiatorController`)**
  Handles radiator venting: rotates panels if excess heat > threshold.

* **`sun_vector`**
  Publishes Sun position for orbital heating.

* **`solar_heat_node`**
  Converts Sun position into actual heat load on nodes.

* **`on_demand_publisher`**
  Diagnostic publisher for one-off checks.

---

##  Launch

All nodes can be started together:

```bash
ros2 launch space_station_thermal_control thermals.launch.py
```

This will bring up:

* Thermal solver
* Coolant action server
* Radiator controller
* Sun vector + solar heating

---

##  Key Interfaces

### Topics

* `/thermal/nodes/state` → Node temperatures
* `/thermal/links/flux` → Heat fluxes between nodes
* `/thermals/diagnostics` → Diagnostic status
* `/tcs/internal_loop_heat` → Internal coolant temperature
* `/tcs/external_loop_a/status` → Ammonia loop status
* `/solar_controller/commands` → Radiator/solar joint commands

### Actions

* `/coolant_heat_transfer` → `Coolant` action (cooling request + feedback loop)

### Services

* `/tcs/radiator_a/vent_heat` → Vent excess heat
* `/wrs/product_water_request` → Request product water
* `/grey_water` → Discard old water

---

##  Example Flow

1. Run the launch file.
2. Watch temperatures rise on `/thermal/nodes/state`.
3. When average temp crosses threshold → `/coolant_heat_transfer` goal is sent.
4. Coolant absorbs heat → publishes feedback (internal temp, ammonia temp, vented heat).
5. If ammonia > 250 kJ → radiator rotates and vents heat.
6. System stabilizes, diagnostics confirm cooling cycle.

