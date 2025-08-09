# space_station_gnc

## Files Specification
| File Name | Purpose | ROS Nodes |
| ---- | ---- | ---- |
| space_station_gnc/src/thruster_matrix.cpp | Define thruster matrix info. | none |
| space_station_gnc/src/physics_motion.cpp | input thruster force and output attitude. | AttitudeDynamicsNode |

## Topics

| Topic Name | published by (node class) | meaning | unit | Type |
| ---- | ---- | ---- | ---- | ---- |
| /gnc/attitude_LVLH | AttitudeDynamicsNode | attitude quaternion | N/A | Quaternion |
| /gnc/angvel_body | AttitudeDynamicsNode | angular velocity | ? | Vector3 |
| /gnc/cmg_del | AttitudeDynamicsNode | ? | ? | ? |
| /gnc/pose_all | AttitudeDynamicsNode | ? | ? | ? |
| /gnc/cmg_h | AttitudeDynamicsNode | ? | ? | ? |
| /gnc/ang_acc | AttitudeDynamicsNode | ? | ? | ? |
| /gnc/bias_thruster_cmd | ControlTorque | thruster force | N ? | ? |
