# **Water Recovery System (WRS)**

## **Overview**

The **Water Recovery System (WRS)** is a core subsystem of the **Environmental Control and Life Support System (ECLSS)**, responsible for **recycling crew urine and wastewater into potable water**. This simulation models the full purification pipeline through ROS 2, including diagnostic monitoring, failure conditions, and integration with downstream systems like the **Oxygen Generation System (OGS)**.

---

## **Core Features**

* ROS 2 Action server for WRS purification cycles
* Realistic simulation of purification stages with temperature and capacity checks
* Asynchronous feedback during each cycle
* Diagnostic publishing for overheating, tank overflow, and system faults
* Product water request and grey water handling via ROS services
* Automatic dispatch to OGS upon success

---

## **Purification Pipeline Simulation**

The WRS node simulates the following purification sequence per cycle:

1. **Urine Processing (UPA):**

   * Simulated with 95% efficiency
   * Overheat failure if temperature exceeds configurable limit

2. **Filtration Unit:**

   * Removes organics, ammonia, and solids
   * 90% efficient, with thermal limits

3. **Ionization Bed:**

   * Final purification and iodine dosing
   * 98% efficient
   * Failure if temperature exceeds limit

4. **Water Tanking & Overflow Detection:**

   * If the product tank exceeds capacity, execution fails

Each purification cycle:

* Processes 5L of urine
* Emits real-time feedback (step, efficiency, tank level)
* Simulates thermal variations

---

## **Topics and Services**

| Type    | Name                         | Description                                |
| ------- | ---------------------------- | ------------------------------------------ |
| Action  | `/water_recovery_systems`    | Main action interface for WRS control      |
| Service | `/wrs/product_water_request` | Returns water from tank if available       |
| Service | `/grey_water`                | Accepts grey water and updates diagnostics |
| Topic   | `/wrs/diagnostics`           | Publishes component-level diagnostics      |
| Topic   | `/wrs/product_water_reserve` | Publishes current reserve water level      |

---

## **Interfacing with OGS**

After successful WRS action completion:

* Purified water (80% of total) is automatically sent as an action goal to the **OGS action server**
* OGS receives this input for oxygen generation
* Logs display O₂ and CH₄ production

---

## **Launch the Node**

Make sure your workspace is built and sourced:

```bash
colcon build --packages-select space_station_eclss
source install/setup.bash
```

Then launch:

```bash
ros2 launch space_station_eclss wrs_systems_v2.launch.py
```

Or run the node directly:

```bash
ros2 run space_station_eclss wrs_action_server
```

---

## **Sending an Action Goal via Terminal**

You can send a test goal using the ROS 2 CLI:

```bash
ros2 action send_goal /water_recovery_systems space_station_eclss/action/WRS "{urine_volume: 10.0}"
```

---

## **Failure Simulation Parameters**

The node supports failure triggers controlled via ROS parameters:

| Parameter                    | Description                                | Default |
| ---------------------------- | ------------------------------------------ | ------- |
| `enable_failure`             | Toggles all simulated failure checks       | `true`  |
| `product_max_capacity`       | Max limit for purified water tank (L)      | `2000`  |
| `product_min_capacity`       | Safety threshold for reserve depletion (L) | `300`   |
| `upa_max_temperature`        | UPA overheat limit (°C)                    | `70.0`  |
| `filter_max_temperature`     | Filter overheat limit (°C)                 | `60.0`  |
| `ionization_max_temperature` | Ionization bed limit (°C)                  | `65.0`  |

---

## **REFERENCE**

![Image 1](https://github.com/user-attachments/assets/90783fa2-7603-4ee5-bd7e-04d7174cbc52)

![Image 2](https://github.com/user-attachments/assets/93306b71-0c29-4237-9f29-42a373c9fe30)

---


