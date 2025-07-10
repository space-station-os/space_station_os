## ** Water Recovery System (WRS)**
### **Overview**
The **Water Recovery System (WRS)** is a key component of the **Environmental Control and Life Support System (ECLSS)** on the **International Space Station (ISS)**. Its purpose is to **recycle wastewater into potable water** for astronauts using **various filtration, oxidation, and purification stages**.

This project **simulates** the **WRS process** using **ROS 2 nodes**, where each node represents a key **filtration, oxidation, or storage** component.

---

## ** Water Recovery Process in Detail**
The **Water Recovery System (WRS)** consists of the following **major components**:

### **1️ Waste & Hygiene Compartment (WHC)**
- **Starting point** of the system.
- Collects **crew urine and hygiene wastewater**.
- **Adds pretreatment chemicals** to prevent microbial growth.
- Sends **processed waste** to the **Waste Collection System**.

### **2️ Waste Collection System**
- Receives **wastewater from WHC**.
- **Accumulates water until a threshold is reached**.
- Sends **collected urine** to the **Urine Processor Assembly (UPA)** for purification.
- **Stops collecting waste** if **UPA is unavailable**.

### **3️ Urine Processor Assembly (UPA)**
- **Distills urine** into **usable water** using a **centrifugal vacuum distillation process**.
- **Prevents new waste intake** while processing.
- **Retains processed water** if the **Water Processor Assembly (WPA) is unavailable**.
- Sends **cleaned water** to the **Filtration Unit**.

### **4️ Filtration Unit**
- **Removes dissolved solids, organics, and ammonia** using:
  - **External Filter Assembly (EFA)**
  - **Particulate Filter**
- **Gradually reduces contaminants** through a **multi-stage filtering process**.
- Passes **filtered water** to the **Catalytic Chamber**.

### **5️ Catalytic Chamber**
- **Oxidizes volatile organic compounds (VOCs)**.
- **Kills bacteria and microbes** using **heat & oxidation**.
- **Further reduces contaminants before final treatment**.
- Sends **sterilized water** to the **Ionization Bed**.

### **6️ Ionization Bed**
- **Final purification step** before storage.
- **Removes remaining contaminants**.
- **Adds iodine** as a **biocide** to prevent microbial growth.
- Sends **final purified water** to the **Product Water Tank**.

### **7️ Product Water Tank**
- Stores up to **100 liters** of **clean drinking water**.
- Supplies water for:
  - **Drinking**
  - **Food rehydration**
  - **Hygiene**
  - **Oxygen Generation System (OGS)**
- **Prevents overflow** by accepting only available tank capacity.
- **Ensures iodine levels** before dispensing water.

---

## ** Node Communication and Topics**
| **Node Name**        | **Incoming Data**                     | **Outgoing Data**                      | **Purpose** |
|----------------------|--------------------------------------|--------------------------------------|------------|
| `whc`               | Crew urine input                     | Waste to `waste_collector`            | Collects urine & hygiene wastewater. |
| `waste_collector`   | Waste from `whc`                     | Processed waste to `upa`              | Stores wastewater and transfers it to UPA. |
| `upa`               | Wastewater from `waste_collector`     | Distillate water to `filter`          | Processes urine to extract water. |
| `filter`            | Distillate from `upa`                | Filtered water to `catalytic_chamber` | Removes organics, ammonia, and particulates. |
| `catalytic_chamber` | Water from `filter`                  | Oxidized water to `ionization`        | Sterilizes water and removes VOCs. |
| `ionization`        | Water from `catalytic_chamber`        | Purified water to `clean_water_tank`  | Final purification and iodine treatment. |
| `clean_water_tank`  | Clean water from `ionization`         | Dispensed water for crew, OGS system  | Stores potable water and sends data. |

---


## ** Running the Simulation**
### **1️ Build and Source the ROS 2 Package**
```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

### **2️ Launch All Nodes**
```bash
ros2 launch space_station_eclss wrs_systems_v2.launch.py
```

### REFERENCES: 

![Image](https://github.com/user-attachments/assets/90783fa2-7603-4ee5-bd7e-04d7174cbc52)


![Image](https://github.com/user-attachments/assets/93306b71-0c29-4237-9f29-42a373c9fe30)