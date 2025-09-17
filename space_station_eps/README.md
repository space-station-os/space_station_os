
#  Primary Electrical Power System (EPS) – Space Station OS

## Overview

The Electrical Power System (EPS) is responsible for collecting, managing, and distributing power across the space station. It mimics the real ISS power architecture, including solar tracking, battery charge/discharge regulation, fault protection, and dynamic routing of power to downstream systems.

The primary components implemented include:

* **SARJ (Solar Alpha Rotary Joint)**
* **Battery Manager**
* **BCDU (Battery Charge/Discharge Unit)**
* **MBSU (Main Bus Switching Unit)**
* **DDCU (Direct Current-to-Direct Current Converter Unit)**

---

##  1. SARJ – Solar Array Tracking

* The SARJ node (`sarj_mock.cpp`) simulates rotation of the solar arrays to track the sun.
* It outputs a **solar voltage estimate** (`/solar/voltage`) that depends on the beta angle and sun position.
* This voltage serves as input to the **BCDU** to decide whether to charge batteries or switch to discharge mode (e.g., during eclipse).

---

##  2. Battery Manager

* Each channel (1–12) has **two ORUs** (Orbital Replacement Units), each with a `BatteryManagementSystem` (BMS).
* The `battery_health.cpp` node:

  * Publishes per-ORU `sensor_msgs/BatteryState` on `/battery/battery_bms_<ID>/health`.
  * Responds to `/charge` and `/discharge` service calls (simulated trigger-based).
  * Applies voltage drop/gain based on activity.
  * Rejects unsafe commands based on voltage limits (e.g., over 120 V or under 70 V).

---

##  3. BCDU – Charge/Discharge Controller

* Implemented as a **ROS 2 action server** in `bcdu_device.cpp`.
* Listens to:

  * Action goals on `/bcdu/operation` (charge or discharge with target voltage).
  * `/solar/voltage` topic to determine when to switch between charge/discharge.
  * Battery status from all 24 BMS instances.
* Parallelizes charge/discharge commands to all healthy ORUs via `/battery/battery_bms_X/charge` and `/discharge`.
* Publishes status to `/bcdu/status` and diagnostics to `/eps/diagnostics`.

Key features:

* Fault isolation if current > 127 A or voltage outside safe range.
* Automatic **safe mode entry** on fault detection.

---

##  4. MBSU – Channel Selection and Routing

* Implemented in `mbsu_distributor.cpp`.
* Subscribes to:

  * `/mbsu/channel_<N>/voltage` — one per EPS channel (voltage derived from BCDU-dispatched ORUs).
* Maintains a live mapping of channel voltages.
* Periodically calls `selectHealthyChannels()`:

  * Picks top 2 healthy channels (voltage > 120 V).
  * Publishes **combined voltage average** to `/ddcu/input_voltage` (simulating DDCU supply).
  * Publishes warning diagnostics if fewer than 2 healthy channels available.

> Note: MBSU does **not** subscribe to BMS health directly anymore — it only relies on **channel voltage** published by BCDU.

---

##  System Flow

```
           [SARJ]
             │
     +───────▼────────+
     │   Solar Power  │
     │    Estimate    │
     +───────┬────────+
             │ /solar/voltage
         [BCDU Action Server]
         /bcdu/operation
        ┌────────┬────────┐
        │        │        │
    /charge  /discharge  Monitor all ORUs
        │        │
 ┌──────▼──┐ ┌────▼────┐
 │ Battery │ │ Battery │  ...
 │  BMS 0  │ │  BMS 1  │  etc.
 └─────────┘ └─────────┘
      │              │
  /health         /health
      ▼              ▼
[MBSU Node]
 └─ Subscribes to:
      /mbsu/channel_<N>/voltage
 └─ Publishes:
      /ddcu/input_voltage (to DDCU)
 └─ Health selection:
      choose top 2 channels
```

---

##  Diagnostics and Fault Handling

* Each node publishes to `/eps/diagnostics` using `diagnostic_msgs/DiagnosticStatus`.
* Faults include:

  * BCDU overcurrent/undervoltage.
  * Battery overcharge/undervoltage.
  * MBSU channel selection failure.

These diagnostics can be monitored in real-time by system status nodes or GUIs.

---

##  Usage Notes

* Channels are configurable via ROS 2 parameters (`num_channels`).
* Simulation parameters (voltage thresholds, current limits) are hardcoded but can be made dynamic.
* Nodes should be launched in the following order for full EPS simulation:

  ```bash
  ros2 run demo_eps sarj_mock
  ros2 run demo_eps battery_health
  ros2 run demo_eps bcdu_device
  ros2 run demo_eps mbsu_device
  ```


<img width="548" height="328" alt="Image" src="https://github.com/user-attachments/assets/804fb5b8-a6a4-4db1-9c3a-d8741a5b4dac" />