
# Air Revitalisation System (ARS)

## Overview

The **Air Revitalisation System (ARS)** is a ROS 2 action-based simulation node that models CO₂ and humidity scrubbing in a closed-loop space habitat. It simulates the CDRA (Carbon Dioxide Removal Assembly) found in the ISS, with key features like adsorbent/desiccant bed temperature monitoring, contaminant management, CO₂ storage regulation, and failure diagnostics.

---

## Functional Description

The `ars_systems_node` provides the `air_revitalisation` action server, which processes incoming goals for removing CO₂, moisture, and contaminants from cabin air.

### Core Features

* **ROS 2 Action Server (`air_revitalisation`)**
  Handles requests to simulate the revitalisation cycle for a given amount of CO₂, moisture, and contaminants. Provides real-time feedback and final results on:

  * Cabin CO₂ level
  * Venting status and amounts
  * Bed activity status (Desiccant & Adsorbent)

* **CO₂ Storage Monitoring**
  Publishes `/co2_storage` topic (`std_msgs/msg/Float64`) to reflect the cumulative CO₂ stored across simulation cycles.

* **Diagnostics and Failures**

  * Publishes `/ars/heartbeat` (`diagnostic_msgs/msg/DiagnosticStatus`) for all critical subsystems and bed temperatures.
  * Injects probabilistic failures related to combustion or contaminant spikes using heartbeat warnings.

* **Contaminant Management**
  Simulates gradual increase in ambient contaminants and triggers alerts when thresholds are breached.

* **CO₂ Service Interface**
  Implements `/ars/request_co2` (`Co2Request` service) to handle CO₂ delivery requests from other systems.

---

## Parameters

| Parameter           | Type     | Description                                     | Default  |
| ------------------- | -------- | ----------------------------------------------- | -------- |
| `sim_time`          | `int`    | Number of simulation steps                      | `10`     |
| `enable_failure`    | `bool`   | Enable random failure injection                 | `true`   |
| `max_co2_storage`   | `double` | Max allowable CO₂ (ppm)                         | `3947.0` |
| `contaminant_limit` | `double` | Max contaminant level before triggering warning | `100.0`  |
| `des*_capacity`     | `double` | Capacity of each desiccant bed                  | `100.0`  |
| `des*_removal`      | `double` | Removal rate per cycle                          | `1.5`    |
| `des*_temp_limit`   | `double` | Max safe temperature for desiccant bed          | `120.0`  |
| `ads*_capacity`     | `double` | Capacity of each adsorbent bed                  | `100.0`  |
| `ads*_removal`      | `double` | CO₂ removal per cycle                           | `2.5`    |
| `ads*_temp_limit`   | `double` | Max safe temp for adsorbent bed                 | `204.0`  |

---

## Data Flow Summary

```text
Cabin Input → Desiccant Bed 1 → Adsorbent Bed 1 → Adsorbent Bed 2 → CO₂ Storage
                                  ↓
                      Desiccant Bed 2 (adds humidity)
```

---

## Heartbeat & Diagnostics

The system uses `diagnostic_msgs/msg/DiagnosticStatus` on `/ars/heartbeat` to signal nominal or failed status of:

* Desiccant and Adsorbent Beds (temperature violation)
* CO₂ Storage (overpressure)
* Contaminant Levels
* Combustion Detector (random failure injection)

---

## Topics

* `/co2_storage` (`std_msgs/msg/Float64`): Current total CO₂ storage.
* `/ars/heartbeat` (`diagnostic_msgs/msg/DiagnosticStatus`): Status reports from all subcomponents.

---

## Services

* `/ars/request_co2` (`space_station_eclss/srv/Co2Request`): Responds to CO₂ draw requests from other systems.

---

## Actions

* `air_revitalisation` (`space_station_eclss/action/AirRevitalisation`):
  Accepts simulation requests and returns results including CO₂ vented, cycles completed, and success status.

---

## Launching

Build the package:

```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

Launch the node:

```bash
ros2 run space_station_eclss ars
```
Sending an action goal 

```bash
ros2 action send_goal /air_revitalisation space_station_eclss/action/AirRevitalisation "{initial_co2_mass: 1800.0, initial_moisture_content: 25.0, initial_contaminants: 5.0}"
```

## REFERENCE

![image](https://github.com/user-attachments/assets/36b7c3ba-2394-422c-a920-b9a2a564ca5f)


