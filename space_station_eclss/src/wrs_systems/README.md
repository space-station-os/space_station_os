## **🚀 Water Recovery System (WRS)**
### **Overview**
The **Water Recovery System (WRS)** is a key component of the **Environmental Control and Life Support System (ECLSS)** on the **International Space Station (ISS)**. Its purpose is to **recycle wastewater into potable water** for astronauts using **various filtration, oxidation, and purification stages**.

This project **simulates** the **WRS process** using **ROS 2 nodes**, where each node represents a key **filtration, oxidation, or storage** component.

---

## **📌 Water Recovery Process in Detail**
The **Water Recovery System (WRS)** consists of the following **major components**:

### **1️⃣ Waste & Hygiene Compartment (WHC)**
- **Starting point** of the system.
- Collects **crew urine and hygiene wastewater**.
- **Adds pretreatment chemicals** to prevent microbial growth.
- Sends **processed waste** to the **Waste Collection System**.

### **2️⃣ Waste Collection System**
- Receives **wastewater from WHC**.
- **Accumulates water until a threshold is reached**.
- Sends **collected urine** to the **Urine Processor Assembly (UPA)** for purification.
- **Stops collecting waste** if **UPA is unavailable**.

### **3️⃣ Urine Processor Assembly (UPA)**
- **Distills urine** into **usable water** using a **centrifugal vacuum distillation process**.
- **Prevents new waste intake** while processing.
- **Retains processed water** if the **Water Processor Assembly (WPA) is unavailable**.
- Sends **cleaned water** to the **Filtration Unit**.

### **4️⃣ Filtration Unit**
- **Removes dissolved solids, organics, and ammonia** using:
  - **External Filter Assembly (EFA)**
  - **Particulate Filter**
- **Gradually reduces contaminants** through a **multi-stage filtering process**.
- Passes **filtered water** to the **Catalytic Chamber**.

### **5️⃣ Catalytic Chamber**
- **Oxidizes volatile organic compounds (VOCs)**.
- **Kills bacteria and microbes** using **heat & oxidation**.
- **Further reduces contaminants before final treatment**.
- Sends **sterilized water** to the **Ionization Bed**.

### **6️⃣ Ionization Bed**
- **Final purification step** before storage.
- **Removes remaining contaminants**.
- **Adds iodine** as a **biocide** to prevent microbial growth.
- Sends **final purified water** to the **Product Water Tank**.

### **7️⃣ Product Water Tank**
- Stores up to **100 liters** of **clean drinking water**.
- Supplies water for:
  - **Drinking**
  - **Food rehydration**
  - **Hygiene**
  - **Oxygen Generation System (OGS)**
- **Prevents overflow** by accepting only available tank capacity.
- **Ensures iodine levels** before dispensing water.

---

## **🔗 Node Communication and Topics**
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

## **📜 Sample Output Log for Each Node**
### **1️⃣ WHC Node (Waste & Hygiene Compartment)**
```plaintext
[INFO] Waste & Hygiene Compartment activated.
[INFO] Urine collection in progress...
[INFO] 1.5 liters of waste collected. Sending to Waste Collector...
```

### **2️⃣ Waste Collection Node**
```plaintext
[INFO] Waste Collection System initialized.
[INFO] Received 1.5 liters of wastewater from WHC.
[INFO] Stored waste level: 5.0 liters.
[INFO] Threshold reached! Sending to Urine Processor Assembly...
```

### **3️⃣ Urine Processor Assembly (UPA)**
```plaintext
[INFO] Urine Processor Assembly started.
[INFO] Received 5.0 liters of wastewater.
[INFO] Distillation in progress...
[INFO] 4.75 liters of clean water extracted (95% efficiency).
[INFO] Sending water to Filtration Unit...
```

### **4️⃣ Filtration Unit**
```plaintext
[INFO] Filtration Unit active.
[INFO] Received 4.75 liters of water from UPA.
[INFO] Removing dissolved solids and ammonia...
[INFO] 90% organics removed. Sending to Catalytic Chamber...
```

### **5️⃣ Catalytic Chamber**
```plaintext
[INFO] Catalytic Chamber activated.
[INFO] Received 4.6 liters of filtered water.
[INFO] Heating water to 150°C for sterilization...
[INFO] Microbial sterilization complete. Sending to Ionization Bed...
```

### **6️⃣ Ionization Bed**
```plaintext
[INFO] Ionization Bed active.
[INFO] Removing final contaminants and adding iodine...
[INFO] Iodine level: 0.5 mg/L.
[INFO] Purified water sent to Product Water Tank...
```

### **7️⃣ Product Water Tank**
```plaintext
[INFO] Product Water Tank initialized.
[INFO] Water level: 80.0 liters.
[INFO] Crew requested 2.0 liters for drinking.
[INFO] Dispensing 2.0 liters. Remaining: 78.0 liters.
[INFO] Publishing tank status to OGS...
```

---

## **🚀 Running the Simulation**
### **1️⃣ Build and Source the ROS 2 Package**
```bash
colcon build --packages-select demo_nova_sanctum
source install/setup.bash
```

### **2️⃣ Launch All Nodes**
```bash
ros2 launch demo_nova_sanctum wrs_systems.launch.py
```

### **3️⃣ Monitor Logs**
To check logs for each node:
```bash
ros2 topic echo /wpa/tank_status
ros2 topic echo /wpa/product_water_tank
```

### **4️⃣ Request Water from Product Water Tank**
```bash
ros2 service call /wpa/dispense_water demo_nova_sanctum/srv/CleanWater "{water: 2.0, iodine_level: 0.5}"
```

### **5️⃣ Check System Performance**
To check CPU & memory usage:
```bash
htop
```

---

### **🛠️ Fixes & Improvements from Logs**
✅ **UPA now correctly retains processed water if WPA is down.**  
✅ **Prevents overflowing the Product Water Tank.**  
✅ **Ensures iodine levels before dispensing water.**  
✅ **Stops waste intake if UPA is unavailable.**  
✅ **Gas bubbles naturally decrease over time.**  

---

🚀 **Now your documentation is fully updated with all the improvements! Let me know if you need any changes!** 🚀

### REFERENCES: 

![Image](https://github.com/user-attachments/assets/90783fa2-7603-4ee5-bd7e-04d7174cbc52)


![Image](https://github.com/user-attachments/assets/93306b71-0c29-4237-9f29-42a373c9fe30)