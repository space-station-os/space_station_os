
## 🧪 README – Air Revitalization System (ARS) | ROS 2

### Overview
The **Air Revitalization System (ARS)** is a ROS 2-based environmental life support simulation for closed-loop air processing in space habitats. It models an advanced 5-stage system inspired by the ISS CDRA and Vozdukh architecture, with realistic adsorption, humidification, and desorption dynamics driven by ROS 2 services and topic streams.

---

### System Architecture

The ARS system is composed of five main ROS 2 nodes:

| Node                | Function                                   |
|---------------------|--------------------------------------------|
| `air_collector_tank` | Collects cabin air, monitors contaminants |
| `desiccant_tank_1`   | Removes moisture and contaminants         |
| `adsorbent_tank_1`   | Adsorbs CO₂ using Langmuir kinetics       |
| `adsorbent_tank_2`   | Desorbs CO₂ and publishes to storage      |
| `desiccant_tank_2`   | Rehumidifies scrubbed air before recirculation |

---

### Data Flow Summary

```text
Air Collector → Desiccant Bed 1 → Adsorbent Bed 1 → Adsorbent Bed 2 → CO₂ Storage
                                         ↓
                             Desiccant Bed 2 (humidifies)
```

---

### Node Communication

#### 🔷 `air_collector_tank`

- **Simulates CO₂, moisture, and contaminant buildup**
- **Conditionally sends air to** `desiccant_bed1`
- Publishes to:
  - `/collector_air_quality` *(AirData)*
  - `/temperature`, `/pipe_pressure`, `/cdra_status`
- Service Client:
  - `/desiccant_bed1`

#### 🔷 `desiccant_tank_1`

- **Dynamically removes moisture/contaminants**
- Sends processed air to `adsorbent_bed1` based on thresholds
- Publishes to:
  - `/desiccant_air_quality`
- Service Server:
  - `/desiccant_bed1`
- Service Client:
  - `/adsorbent_bed1`

#### 🔷 `adsorbent_tank_1`

- **Performs Langmuir-style CO₂ adsorption**
- Uses dynamic `k_ads` based on `mode_of_operation`
- Sends captured CO₂ to `adsorbent_bed2` if buffer full
- Publishes to:
  - `/adsorbent_air_quality`, `/cdra_status`
- Service Server:
  - `/adsorbent_bed1`
- Service Client:
  - `/adsorbent_bed2`, `/desiccant_bed2`

#### 🔷 `adsorbent_tank_2`

- **Heats and desorbs CO₂**
- Publishes accumulated CO₂ to `/co2_storage`
- Publishes to:
  - `/cdra_status`, `/co2_storage` *(Float64)*
- Service Server:
  - `/adsorbent_bed2`

#### 🔷 `desiccant_tank_2`

- **Reintroduces humidity to dry air stream**
- Publishes reconditioned air to:
  - `/desiccant_air_quality`
- Service Server:
  - `/desiccant_bed2`

---

### Launching the System

Make sure you've built the workspace:

```bash
colcon build --packages-select demo_nova_sanctum
source install/setup.bash
```

Launch all components:

```bash
ros2 launch demo_nova_sanctum ars_systems_v4.launch.py
```

Or run each manually:

```bash
ros2 run demo_nova_sanctum air_collector_tank
ros2 run demo_nova_sanctum desiccant_tank_1
ros2 run demo_nova_sanctum adsorbent_tank_1
ros2 run demo_nova_sanctum adsorbent_tank_2
ros2 run demo_nova_sanctum desiccant_tank_2
```

---

### Modes of Operation

All nodes respond dynamically to a global ROS parameter `mode_of_operation`, which adjusts processing rates. Supported modes include:

- `idle`
- `standby`
- `exercise`
- `emergency`
- `biological_research`
- `eva_repair`

---

### Topics Summary

| Topic Name              | Message Type                 | Publisher            |
|-------------------------|------------------------------|----------------------|
| `/collector_air_quality`| `AirData`                    | air_collector        |
| `/desiccant_air_quality`| `AirData`                    | desiccant_tank_1/2   |
| `/adsorbent_air_quality`| `AirData`                    | adsorbent_tank_1     |
| `/co2_storage`          | `Float64`                    | adsorbent_tank_2     |
| `/cdra_status`          | `CdraStatus`                 | all stages           |

---

### Services Summary

| Service Name          | Server              | Client                   |
|-----------------------|---------------------|--------------------------|
| `/desiccant_bed1`     | desiccant_tank_1    | air_collector            |
| `/adsorbent_bed1`     | adsorbent_tank_1    | desiccant_tank_1         |
| `/adsorbent_bed2`     | adsorbent_tank_2    | adsorbent_tank_1         |
| `/desiccant_bed2`     | desiccant_tank_2    | adsorbent_tank_1         |

---

### Sample `ars_sys.yaml`

```yaml
air_collector_node:
  ros__parameters:
    flow_rate: 28.0  # Flow rate in SCFM
    co2_intake: 1.04 # CO2 intake in mmHg
    crew_onboard: 4
    cabin_pressure: 14.7  # Cabin pressure in PSI
    temperature_cutoff: 450.0  # Temperature cutoff in Celsius
    max_crew_limit: 6
    power_consumption: 1.0  # Power in kW
    tank_capacity: 1000.0  # Air tank capacity in grams
    system_name: "demo_nova_sanctum"
    mode_of_operation: "standby"

    co2_threshold: 500.0
    moisture_threshold: 70.0
    contaminants_threshold: 30.0

    temp_kp: 0.1
    temp_ki: 0.01
    temp_kd: 0.005
    press_kp: 0.1
    press_ki: 0.01
    press_kd: 0.005


/desiccant_bed_1:
  ros__parameters:
    moisture_removal_rate: 0.20
    contaminant_removal_rate: 0.20
    emergency_threshold: 5.0

/desiccant_bed_2:
  ros__parameters:
    humidification_rate: 1.5
    emergency_threshold: 5.0

/adsorbent_bed_1:
  ros__parameters:
    co2_removal_efficiency: 0.95
    co2_to_space_ratio: 0.40
    desired_temperature: 420.0
    temperature_tolerance: 30.0
    kp: 0.6
    kd: 0.15
    co2_adsorption_rate_constant: 0.015  # Langmuir k_ads
    co2_capacity: 850.0  # Total CO₂ adsorption capacity

/adsorbent_bed_2:
  ros__parameters:
    desorption_temperature: 400.0
    co2_desorption_rate_constant: 0.05  # Langmuir desorption rate

```

---
## REFERENCE

![image](https://github.com/user-attachments/assets/36b7c3ba-2394-422c-a920-b9a2a564ca5f)

### Future Improvements
- Add feedback control on inter-node latency
- Integrate data logging (CO₂ levels, throughput)
- Deploy to a virtual ISS environment with visualization

