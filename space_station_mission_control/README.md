# SSOS Ground Control System (Open MCT + ROS 2)

https://github.com/user-attachments/assets/450c3e84-f83e-40d6-a149-aaf5e2e7f394


This dashboard visualizes telemetry from ROS 2 nodes using **Open MCT** and **rosbridge_server**. It is tailored for the Space Station OS (SSOS) project, including:

- **ECLSS** (Environmental Control and Life Support System)
- **ATCS** (Thermal Control)
- **GNC** (Guidance, Navigation, and Control)

---

##  Setup Instructions

### 1. Clone and Build `mission_control`

```bash
git clone --recurse-submodules https://github.com/space-station-os/demo_mission_control.git

cd mission_control/openmct-ros
npm install
npm run build:example
npm start
````

> This starts Open MCT at:
> **[http://localhost:9097/](http://localhost:9097/)**

---

### 2. Install and Launch `rosbridge_server`

Install:

```bash
sudo apt install ros-humble-rosbridge-server
```

Launch:

```bash
tmux new -s rosbridge -d  "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
```
>Note : Here assuming tmux is already installed 

This runs the WebSocket bridge at:

```
ws://localhost:9090
```

---

##  Import Your Custom Dashboard

Once Open MCT is running:

1. Open your browser at:
   **[http://localhost:8080](http://localhost:8080)**

2. In the top-left menu, click:
   **Create → Import from JSON**

3. Choose your dashboard file, e.g.:
   `SSOS GCS.json`

4. It will load your folder structure and all visual telemetry (ECLSS, ATCS, GNC).

---

##  Requirements

* ROS 2 (Humble or newer)
* `rosbridge_server`
* Node.js (v16–18 recommended)
* Modern browser (Chrome/Firefox)

---

You can now launch any subsystem and create your own mission control dashboard.

