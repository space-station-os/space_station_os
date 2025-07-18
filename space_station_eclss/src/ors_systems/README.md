# Oxygen Generation System (OGS)

## Overview

The **Oxygen Generation System (OGS)** models a closed-loop life support component similar to the one used on the **International Space Station (ISS)**. It simulates core oxygen recovery mechanisms including:

* Water electrolysis
* Hydrogen management
* Sabatier reaction for methane venting and water reclamation
* Real-time diagnostics, failure injection, and grey water transfer

This ROS 2-based implementation includes robust goal handling, diagnostics publishing, and failure simulation.

---

## Key Features

* **Action-based control** via `oxygen_generation` action server
* **Multi-stage process**: product water → electrolysis → Sabatier → venting & feedback
* **Dynamic diagnostics** for subsystems like O₂ storage, electrolysis, and Sabatier reactor
* **Failure simulation** for overpressure/underpressure scenarios
* **Grey water output** to water recovery

---

## System Diagram

```text
      +----------------+
      | Product Water  |
      |   Request      |
      +-------+--------+
              |
              v
      +--------------------+
      |   Electrolysis     |
      | Generates O₂, H₂   |
      +--------+-----------+
               |
     +---------+---------+
     |     Sabatier      |
     | CO₂ + H₂ → CH₄ + H₂O
     +----+-------+------+
          |       |
      CH₄ vent  Grey Water → WRS
```

---

## Launch Instructions

Build and source the workspace:

```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

Then run the OGS node:

```bash
ros2 run space_station_eclss ogs_system
```

---

## Sending an Action Goal

To manually trigger oxygen generation from the terminal, use the `ros2 action send_goal` command:

```bash
ros2 action send_goal /oxygen_generation space_station_eclss/action/OxygenGeneration "{input_water_mass: 2.5, iodine_concentration: 0.3}"
```

---

## Node Details

| Interface Type     | Name                         | Purpose                                |
| ------------------ | ---------------------------- | -------------------------------------- |
| **Action Server**  | `/oxygen_generation`         | Accepts requests for oxygen production |
| **Service Client** | `/wrs/product_water_request` | Requests potable water                 |
| **Service Client** | `/ars/request_co2`           | Requests CO₂ from ARS                  |
| **Service Client** | `/grey_water`                | Sends grey water to recovery           |
| **Service Server** | `/ogs/request_o2`            | Provides oxygen on request             |
| **Publisher**      | `/o2_storage`                | Current O₂ amount (g)                  |
| **Publisher**      | `/methane_vented`            | Total CH₄ vented (g)                   |
| **Publisher**      | `/ogs/diagnostics`           | System health diagnostics              |

---

## Parameters

| Parameter             | Default   | Description                                 |
| --------------------- | --------- | ------------------------------------------- |
| `enable_failure`      | `false`   | Simulate system failures                    |
| `electrolysis_temp`   | `100.0`   | Operating temperature of electrolysis       |
| `o2_efficiency`       | `0.95`    | Efficiency of O₂ generation from H₂O        |
| `sabatier_efficiency` | `0.75`    | CH₄ output efficiency from Sabatier reactor |
| `sabatier_temp`       | `300.0`   | Sabatier reaction temperature (K)           |
| `sabatier_pressure`   | `1.0`     | Sabatier reactor pressure (atm)             |
| `min_o2_capacity`     | `100.0`   | Minimum safe O₂ capacity (g)                |
| `max_o2_capacity`     | `10000.0` | Maximum safe O₂ capacity (g)                |

---

## Expected Behavior

* **Water is requested** from WRS and electrolyzed to generate O₂ and H₂
* **CO₂ is fetched** from ARS to drive the Sabatier reaction
* **CH₄ is vented**, **grey water is recycled** to WRS
* Feedback and diagnostics are published at each stage

---

## Image Reference

![OGS Diagram](https://github.com/user-attachments/assets/3b07155a-b415-404f-ba10-e230ec83b447)

> *Courtesy: Elizabeth M. Bowman et al., The Boeing Company, Huntsville, AL*
