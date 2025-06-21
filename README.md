# **Space Station OS – Setup & Demo Guide**


<p align="center">
  <img src="https://github.com/user-attachments/assets/4cfe5156-7282-4c26-aa7b-324bb8c1196b" width="100%" />
</p>


[![ROS 2 Humble CI](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml/badge.svg)](https://github.com/space-station-os/space_station_os/actions/workflows/ros2_humble_ci.yml)


##  Prerequisites

Before starting, make sure you have the following:

* **Operating System:** Ubuntu 22.04
* **ROS 2 Distribution:** ROS 2 Humble (Desktop Install)
  [→ Official ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

##  Installation Steps

### 1. Prepare a ROS 2 workspace

If you're new to ROS 2, create a workspace:

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src
```

### 2. Clone the `space_station_os` super-repository with submodules

```bash
git clone --recurse-submodules https://github.com/space-station-os/space_station_os.git
cd space_station_os
```

> If you've already cloned it, initialize submodules manually:

```bash
git submodule update --init --recursive
```

### 3. Build the workspace

Go back to the workspace root and build everything:

```bash
cd ~/ssos_ws
colcon build --symlink-install
source install/setup.bash
```

---

>  Tip: Always source your workspace before running ROS 2 nodes:
>
> ```bash
> source ~/ros2_ws/install/setup.bash
> ```

---

##  Running Demo 1: ISS Incident Simulation (Nauka & Others)

This demo shows the attitude control and fault management systems of a space station through three different crisis scenarios.

###  Terminal Setup Overview

You’ll need **3 terminals** for full functionality:

---

###  Terminal 1: Run the Incident Scenario

Choose one of the following scenarios:

* **Demo 1a:** Nauka incident simulation (July 2021)
  *(moderate severity, real-world based)*

  ```bash
  ros2 run space_station_gnc demo1a_nauka_incident_estimate
  ```

* **Demo 1b:** Hypothetical engine crisis
  *(high severity test case)*

  ```bash
  ros2 run space_station_gnc demo1b_crisis_mainengine
  ```

* **Demo 1c:** Minor perturbation
  *(ideal for testing CMG control logic)*

  ```bash
  ros2 run space_station_gnc demo1c_small_incident
  ```

---

###  Terminal 2: Launch the GNC Core System

This runs the core control logic and dynamics for all scenarios:

```bash
ros2 launch space_station_gnc gnc_core.launch.py
```

No interaction is needed in this terminal once launched.

---

###  Terminal 3: Launch RViz for Visualization

```bash
ros2 launch space_station_gnc gnc_rviz.launch.py
```

#### RViz Setup:

1. Set `Fixed Frame` to `world`
2. Click `Add` → Select `RobotModel`
3. Set `Description Topic` to `/robot_description`
4. Zoom out if needed to see the ISS or your custom model

#### Optional: Use Your Own URDF

To view a custom space station model:

* Place your URDF in:
  `~/ros2_ws/src/space_station_os/space_station_gnc/urdf`
* Example provided: `SD_SpaceStation_Ver05.urdf`

To configure which URDF to use:

* Modify:
  `~/ros2_ws/src/space_station_os/space_station_gnc/launch/gnc_core.launch.py`

---

###  Running the Scenario

Return to **Terminal 1**, follow the on-screen scenario instructions, and observe the response in **RViz** as the space station reacts dynamically to simulated faults or perturbations.

---



### **Environmental Control and Life Support Systems (ECLSS)**
Environmental Control and Life Support Systems (ECLSS) are essential for sustaining human life in space by providing a controlled environment that includes air revitalization, water recovery, and waste management. This document serves as an overview of ECLSS and provides links to specific subsystems implemented as part of this project.

## **Overview of ECLSS Subsystems**
ECLSS consists of multiple interconnected subsystems to maintain habitable conditions for astronauts:

- **Air Revitalization System (ARS):** Handles **CO₂ removal, moisture control, and contaminant filtration** to maintain breathable air.
- **Oxygen Recovery System (ORS):** Converts **water into oxygen** through electrolysis and uses **hydrogen recovery** to form a closed-loop system.
- **Water Recovery and Balance Systems:** Processes crew urine, atmospheric condensation, and Sabatier-produced water for reuse.
- **Temperature and Humidity Control:** Regulates cabin conditions to ensure thermal comfort and moisture control.

## **Available Subsystem Implementations**
Below are the specific subsystems implemented as part of this project. Click on the links to access their respective documentation.

### **1. Air Revitalization System (ARS)**
The ARS is responsible for maintaining breathable air by removing CO₂, moisture, and contaminants from the cabin environment. The system consists of multiple ROS2 nodes working together to simulate air purification onboard the **International Space Station (ISS)**.

🔗 [Read the full ARS documentation](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/ars_systems/README.md)

### **2. Oxygen Recovery System (ORS)**
The ORS simulates the oxygen generation process used on the ISS. It leverages **electrolysis, Sabatier reaction, and deionization** to create a closed-loop system that efficiently recycles oxygen from water.

🔗 [Read the full ORS documentation](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/ors_systems/README.md)

### **2. Water Recovery And Purification Systems (WRPS)**
The WRS system purifies the waste accumulated from the crew and converts it into potable water that is fit for consumption. Some amount of water is also used to get oxygen by electrolysis

🔗 [Read the full WRS documentation](https://github.com/space-station-os/space_station_os/blob/main/space_station_eclss/src/wrs_systems/README.md)



#### To launch the Systems:
---

```sh
ros2 launch space_station_eclss eclss.launch.py
```
# SPACE STATION GAZEBO 

Haven-1 Model (downgraded)

[spacestation_full.webm](https://github.com/user-attachments/assets/39a9498a-2918-42c6-84a6-8373325f9fbe)

---

HAVEN-2 Model 
[Haven-2.webm](https://github.com/user-attachments/assets/3a8192e6-1982-4684-8276-a00e9de465c0)

```sh
ros2 launch space_station_description gazebo.launch.py
```

#### To run the teleoperation 
---

```sh
ros2 run space_station_description mux
ros2 run space_station_description teleop
```



## Interested to contribute? 
See the project backlog https://github.com/orgs/space-station-os/projects/2/views/1 

