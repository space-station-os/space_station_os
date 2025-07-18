# **Human Simulation GUI for ECLSS**

The **Human Simulation GUI** is a PyQt-based graphical interface designed to simulate astronaut life support interactions within a closed-loop **Environmental Control and Life Support System (ECLSS)** aboard a spacecraft or station. This GUI drives crew activity logic and interfaces with underlying ROS 2 action and service servers for the following systems:

* **Air Revitalisation System (ARS)**
* **Water Recovery System (WRS)**
* **Oxygen Generation System (OGS)**

---

## **Modes of Operation**

Upon launch, the GUI allows selection of one of three simulation modes:

### **1. User Mode**

* Minimal configuration
* Suitable for general demonstrations
* Sends standard goals to life support systems

### **2. Developer Mode**

* Allows parameter tuning
* Logs additional debug output
* Exposes underlying system behavior

### **3. Astronaut Mode**

* Simulates a full crew's daily physiological needs
* Evaluates system response across multiple days and events
* Includes failure conditions and recovery

---

## **System Architecture**

### **Node: `crew_simulation_node`**

This node powers the GUI and simulates crew metabolic demands by:

* Sending **action goals** to ARS and WRS
* Calling **services** for oxygen and water from OGS and WRS
* Subscribing to O₂ and water reserve levels to decide when to trigger actions

### **Topics/Subscribers**

| Topic                        | Message Type       | Purpose                     |
| ---------------------------- | ------------------ | --------------------------- |
| `/o2_storage`                | `std_msgs/Float64` | Receives O₂ reserve updates |
| `/wrs/product_water_reserve` | `std_msgs/Float64` | Receives water tank status  |

### **Action Clients**

| Action                   | Purpose                          |
| ------------------------ | -------------------------------- |
| `air_revitalisation`     | Removes CO₂ and contaminants     |
| `water_recovery_systems` | Purifies urine into usable water |

### **Service Clients**

| Service                     | Purpose                  |
| --------------------------- | ------------------------ |
| `ogs/request_o2`            | Requests oxygen from OGS |
| `wrs/product_water_request` | Requests drinking water  |

---

## **Execution Instructions**

### **1. Build the Workspace**

```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

### **2. Launch Full ECLSS Simulation with GUI**

```bash
ros2 launch space_station_eclss eclss.launch.py
```

This will launch:

* The Human Simulation GUI
* ARS, WRS, and OGS system nodes
* All required ROS 2 interfaces for simulation

---

## **GUI Overview**

* **Image Banner:** Displays an SSOS image if available
* **Mode Selector:** Buttons for User, Developer, and Astronaut modes
* **Log Box:** Shows simulation output, status messages, and feedback
* **Sim Controller (per mode):** Each mode opens a specialized window with parameter controls and real-time simulation feedback

---

## **Simulation Behavior**

For each simulation event (e.g., per crew member per day):

1. **ARS Goal Sent:** Simulates air scrubbing for CO₂ and moisture
2. **WRS Goal Sent:** Processes urine to produce clean water
3. **O₂ Requested:** From OGS if available
4. **Water Requested:** From WRS if tank has sufficient volume

### Parameters (configurable)

| Parameter                 | Description                         |
| ------------------------- | ----------------------------------- |
| `crew_size`               | Number of crew members              |
| `events_per_day`          | Simulation steps per day            |
| `number_of_days`          | Total mission duration              |
| `mode`                    | Activity level (`rest`, `exercise`) |
| `calorie_intake`          | Per person per day                  |
| `potable_water_intake`    | Per person per day (liters)         |
| `ars_failure_enabled`     | ARS failure mode                    |
| `ogs_enable_failure`      | OGS failure mode                    |
| `ogs_o2_efficiency`       | Electrolysis efficiency             |
| `ogs_sabatier_efficiency` | CO₂ to CH₄ conversion efficiency    |
| `wrs_max_capacity`        | Max tank capacity for WRS           |

---

## **Dependencies**

* ROS 2 (Humble or compatible)
* rclpy, actionlib, std\_msgs, diagnostic\_msgs
* PyQt5

Install dependencies (if needed):

```bash
pip install pyqt5
```

---

## **Assets**

Ensure the following image asset is present:

```
space_station_eclss/assets/ssos.png
```
### SSOS ECLSS SIMULATOR

<img width="498" height="499" alt="image" src="https://github.com/user-attachments/assets/a990a22e-2526-4b09-8075-2c2c3617ef4b" />

> In case of failure, you will get something like this:

<img width="419" height="520" alt="image" src="https://github.com/user-attachments/assets/056780af-4656-4769-9f13-fda7b0e228dd" />

