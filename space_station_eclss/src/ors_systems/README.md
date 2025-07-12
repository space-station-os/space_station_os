# **Oxygen Recovery System (ORS)**

This project simulates the **Oxygen Recovery System (ORS)** used on the **International Space Station (ISS)**. The system operates in a closed-loop environment and consists of multiple processes, including **water accumulation, deionization, electrolysis, and the Sabatier reaction** to ensure sustainable oxygen production for astronauts.

--- 

## **Execution Instructions**

### **1. Start ROS2 system:**
```sh
ros2 launch space_station_eclss ogs_systems_v2.launch.py
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

