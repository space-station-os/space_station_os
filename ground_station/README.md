# README
This README provides the deployment instructions for system integration from the GNC and ground station perspectives. Included modules/technologies start from the GNC subsystem demo and are extended with fault injection, data handling, communication to Yamcs (mission control software), database storage (MongoDB), and a virtual Ground Control Station (in React).

Several communication algorithms are utilized (e.g., FASTApi, http binding/serving, etc.), so please check other documentation for a deeper understanding if necessary.

---- 

  Deployment Instructions
----

1) Global Setup (applies to all terminals)
   1. `source /opt/ros/humble/setup.bash`
   2. `cd space_station_os`
   3. `colcon build`
   4. `source install/setup.bash`

2) GNC-Related Demo Nodes (each in its own terminal)
   - `ros2 run space_station_gnc demo1a_nauka_incident_estimate`
   - `ros2 launch space_station_gnc launch_gnc.py`
   - `ros2 run rviz2 rviz2`

3) Ground Control Modules (from space_station_os/ground_station/src/scripts)
   - `python3 iss_path.py`
   - `python3 iss_visualizer.py`
   - `python3 GCS_data_handler.py`
   - `python3 imu_fault_injector.py`

4) Yamcs Integration (from space_station_os/ground_station/src/ssos_gcs_yamcs)
   - `python3 ros_yamcs_bridge.py`
   - `python3 simulator.py`
   - `mvn yamcs:run`

5) MongoDB Integration
   1. `cd /mongodb_data:`
       - `mongosh`
   2. `cd space_station_os/ground_station/src/backend`
       - `uvicorn api_backend:app --host 0.0.0.0 --port 5000 --reload`
       - `python3 gcs_mongo_db_node.py`

6) React (Virtual Ground Control Station)
   1. `python3 -m http.server 8000 --bind 127.0.0.1`
   2. `space_station_os/ground_station/src/ssos_gcs_react`
        - `npm start`

---
  Node / Function Descriptions
---

demo1a_nauka_incident_estimate
- Demonstrates or tests the GNC’s ability to detect and estimate an incident on the Nauka module.

launch_gnc.py
- Launch file for coordinating and running multiple GNC-related nodes.

RViz2
- ROS visualization tool for monitoring real-time 3D environments and sensor data.

iss_path.py
- Simulates or calculates the International Space Station (ISS) orbital path.

iss_visualizer.py
- Provides a 2D/3D representation of the ISS orbit for visualization in the ground station.

GCS_data_handler.py
- Manages and processes telemetry or sensor data within the Ground Control Station (GCS).

imu_fault_injector.py
- Injects simulated faults into IMU data for testing system resilience and fault-tolerance.

ros_yamcs_bridge.py
- Bridges data between ROS topics and YAMCS (for telemetry/mission control).

simulator.py
- Generates simulated telemetry or mission data for testing YAMCS integration.

mvn yamcs:run
- Starts the YAMCS server.

mongosh
- MongoDB shell, used for connecting to and managing the MongoDB instance.

uvicorn api_backend:app
- Runs the FastAPI backend for handling RESTful APIs and database interactions.

gcs_mongo_db_node.py
- ROS node that interfaces with MongoDB, enabling data storage/retrieval for GCS.

python3 -m http.server 8000
- Hosts a basic local HTTP server to serve built web assets (if available).

npm start
- Launches the React development server for the Virtual Ground Control Station (UI).

---- 
  End of Instructions
----