# 🧠 STL Monitoring and Dashboard

### Overview
This module implements a **TeLEx-based Signal Temporal Logic (STL) Monitor** that tracks safety-critical air quality bounds within a ROS 2 life support system. The system validates that parameters like **CO₂ levels, moisture, and contaminants** remain within formally derived thresholds at runtime.

If the signal violates a specification for >10 consecutive times, a **CODE RED** is triggered — alerting astronauts and displaying an emergency popup in the mission dashboard.

---

### STL Specification (TeLEx Derived)

The STL specifications were extracted from trajectory learning experiments and optimized using **Differential Evolution**. Each subsystem has its own invariant property:

| Subsystem         | STL Formula                               | Bounds           |
|------------------|--------------------------------------------|------------------|
| Air Collector     | □(6.02 ≤ co₂_mass ≤ 197.09)               | [6.02, 197.09]   |
| Desiccant Bed     | □(moisture ≤ 153.21 ∧ contaminants ≤ 35.08)| [0, 153.21], [0, 35.08] |
| Adsorbent Bed     | □(2.26 ≤ co₂_mass ≤ 59.53)                | [2.26, 59.53]    |

---

### STLMonitor Node (C++ ROS 2)

#### 🔍 Features:
- Monitors all three `AirData` publishers: `/collector_air_quality`, `/desiccant_air_quality`, `/adsorbent_air_quality`
- Computes **robustness**:  
  `r = min(value - a, b - value)`
- Tracks consecutive violations
- Triggers `CODE_RED` after 10 violations
- Publishes STL health to:
  - `/stl_monitor/status` (JSON string for GUI)
  - `/crew_alert` (for emergency escalation)

#### 🚨 Emergency Escalation:
If a parameter enters `CODE_RED`, the monitor:
- Logs via `RCLCPP_FATAL`
- Sends a broadcast to `/crew_alert`
- Triggers an emergency pop-up in the Vue.js dashboard

---

### STL Dashboard Visualization (Vue.js + roslibjs)

#### 📊 Realtime Components:
- `StatusHUD.vue`: Displays live **status, temp, pressure, mode**
- `EmergencyPopup.vue`: Pops up if any STL monitor enters `CODE_RED`
- `App.vue`: Parses STL status JSON and passes it to all components
- `roslibjs` + `rosbridge_websocket` used for live updates from ROS 2

#### 🌐 STL Status Payload Format:
```json
{
  "collector": "PASS",
  "desiccant_moisture": "PASS",
  "desiccant_contaminants": "PASS",
  "adsorbent": "CODE_RED"
}
```

This format is parsed live in the browser, and animated status indicators update in real-time with blinking lights or modal alerts.

---

### Topics and Architecture

| ROS Topic              | Type             | Description                             |
|------------------------|------------------|-----------------------------------------|
| `/collector_air_quality` | `AirData`      | CO₂, moisture, contaminants from Air Collector |
| `/desiccant_air_quality` | `AirData`      | Output after moisture removal           |
| `/adsorbent_air_quality` | `AirData`      | Output after CO₂ adsorption             |
| `/stl_monitor/status`   | `std_msgs/String` | JSON for GUI HUD                        |
| `/crew_alert`           | `std_msgs/String` | Trigger for emergency popup             |

---

### Launch Instructions

Make sure your ROS environment is built:

```bash
colcon build --packages-select demo_nova_sanctum
source install/setup.bash
```

Launch everything with `tmux` automation:

```bash
./ARS.sh
```

That script will:
1. Launch ARS system nodes
2. Run `safety` (STLMonitor)
3. Run `iss_simulator.py`
4. Launch `rosbridge_server`
5. Start your Vue.js dashboard (`npm run serve`)

---

### STL Alert Lifecycle

| Phase           | STL Status | Visuals                       | Action Triggered                   |
|----------------|------------|-------------------------------|------------------------------------|
| Nominal        | `OK`       | Static green HUD              | No alert                           |
| Out-of-bounds  | `FAIL`     | Yellow LED / recovery counter | Starts recovery timer              |
| 10 violations  | `CODE_RED` | Red LED + Emergency Modal     | `/crew_alert` + ROS Fatal + Popup  |

---

### Final Presentation

To demonstrate your STL system:
- Run the simulator to slowly inject a violation
- Watch `StatusHUD` flip to `FAIL`, and escalate
- Observe the animated emergency popup and blinking indicators

[\[Screencast from 04-15-2025 11:33:57 PM.webm\](https://github.com/user-attachments/assets/253b583e-64b0-4c3f-913c-0f81c82d41d2)
](https://github.com/user-attachments/assets/c8de592b-b644-4477-8488-3cd671bd3cdc)

