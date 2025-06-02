# **Oxygen Recovery System (ORS)**

This project simulates the **Oxygen Recovery System (ORS)** used on the **International Space Station (ISS)**. The system operates in a closed-loop environment and consists of multiple processes, including **water accumulation, deionization, electrolysis, and the Sabatier reaction** to ensure sustainable oxygen production for astronauts.

---
## **System Overview**

The ORS consists of the following primary nodes:

1. **Water Pump (`water_pub.cpp`)** - Simulates incoming water accumulation and sends water to deionization when the tank is full.
2. **Deionization Bed (`deionization_chamber.cpp`)** - Removes contaminants, iodine, and gas bubbles before sending the water to the electrolysis chamber.
3. **Electrolysis Chamber (`electrolysis.cpp`)** - Splits water into **hydrogen (H₂) and oxygen (O₂)**.
4. **Sabatier Reactor (`sabatier.cpp`)** - Uses the **hydrogen from electrolysis and carbon dioxide (CO₂) from the crew** to produce **methane (CH₄) and water (H₂O)**.

---
## **System Communication & Data Flow**

Each node communicates via ROS2 **services** and **topics** to form a continuous loop for oxygen and water recycling.

### **1. Water Pump (`water_pub.cpp`)**
- **Accumulates water over time** at a fixed rate.
- **When the tank is full (100L),** sends a **service request** to the deionization bed (`/deionization_chamber`).
- **Waits for a response** before emptying the tank and restarting accumulation.

**ROS Interfaces:**
| **Service Client** | **Service Server** |
|--------------------|--------------------|
| `/deionization_chamber` | `/water_request` |

---

### **2. Deionization Bed (`deionization_chamber.cpp`)**
- **Removes iodine, contaminants, and gas bubbles** from water.
- **If gas bubbles are detected,** diverts water via a three-way valve.
- **Once purified,** sends water to the electrolysis chamber via a **service request** (`/electrolysis`).
- **Waits for a response** before accepting new water from the water pump.

**ROS Interfaces:**
| **Service Client** | **Service Server** |
|--------------------|--------------------|
| `/electrolysis` | `/deionization_chamber` |

---

### **3. Electrolysis Chamber (`electrolysis.cpp`)**
- **Receives purified water** and processes it via electrolysis.
- **Splits H₂O into hydrogen and oxygen.**
  - **Hydrogen is sent to the Sabatier reactor**.
  - **Oxygen is released into the station's atmosphere**.
- **Water levels decrease by 30% per cycle** (controlled by depletion factor).
- **If water reaches 2L or less,** electrolysis stops and waits for new water.

**ROS Interfaces:**
| **Service Server** | **Published Topics** |
|--------------------|--------------------|
| `/electrolysis` | `/electrolysis_output` (H₂ and O₂ data) |

---

### **4. Sabatier Reactor (`sabatier.cpp`)**
- **Receives hydrogen from electrolysis** (`/electrolysis_output`).
- **Receives CO₂ from a simulated atmosphere** (`/processed_co2`).
- **Runs the Sabatier reaction:**
  - **CO₂ + 4H₂ → CH₄ + 2H₂O**
  - Produces **methane (CH₄)** (vented to space) and **water (H₂O)** (sent back to electrolysis).
- **Uses PID controllers to regulate temperature, pressure, and gas flow.**
- **Publishes reactor outputs** (`/sabatier`).

**ROS Interfaces:**
| **Subscribed Topics** | **Published Topics** |
|----------------------|----------------------|
| `/electrolysis_output` (H₂ data) | `/sabatier` (CH₄ & H₂O data) |
| `/processed_co2` (CO₂ data) |  |

---

## **ROS2 System Architecture**

```plaintext
[ Water Pump ]
    ↓ (Request: /deionization_chamber)
[ Deionization Bed ]
    ↓ (Request: /electrolysis)
[ Electrolysis Chamber ]
    ↓ (Publishes: /electrolysis_output)
[ Sabatier Reactor ]
    → (CH₄ to space, H₂O to electrolysis)
```

---
## **ROS2 Parameters**

| **Parameter** | **Default Value** | **Node** | **Description** |
|--------------|------------------|---------|----------------|
| `efficiency_factor` | `0.95` | Electrolysis | Electrolysis efficiency |
| `required_pressure` | `50.0` | Electrolysis | Required pressure for electrolysis |
| `depletion_factor` | `0.3` | Electrolysis | Water depletion per cycle |
| `accumulation_rate` | `5.0` | Water Pump | Water accumulation per cycle |

---

## **Execution Instructions**

### **1. Start ROS2 system:**
```sh
ros2 launch demo_nova_sanctum or_system.launch.py
```

### **2. Dynamically Set Parameters:**
```sh
ros2 param set /electrolysis_node efficiency_factor 0.98
ros2 param set /electrolysis_node required_pressure 55.0
ros2 param set /water_service accumulation_rate 6.0
```

### **3. Monitor Logs:**
```sh
ros2 topic echo /electrolysis_output
ros2 topic echo /sabatier
```

---
## **System Behavior: Expected Outputs**
- **Water fills in cycles → 100L → 70L → 49L → ... → 0L**
- **Electrolysis stops at 0L and waits for new water.**
- **Hydrogen is converted in the Sabatier reactor to recover water.**
- **Methane is vented to space.**
- **Water recirculates back to electrolysis.**

---
## **Image Reference**
This diagram illustrates the **Oxygen Recovery System** architecture.

![image](https://github.com/user-attachments/assets/3b07155a-b415-404f-ba10-e230ec83b447)


Courtesy: Analysis of Resin Samples From a Return-To -Ground Inlet 
Deionizing Bed for the ISS Oxygen Generation System 
Elizabeth M. Bowman0F1, Danielle N. Bowman1F2, Darren S. Dunlap2F3, David A. Jackson3F4, and Natalee E. Weir4F5 
The Boeing Company, Huntsville, AL, 35824

