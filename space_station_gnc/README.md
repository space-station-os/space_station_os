# space_station_gnc

## Files Specification
| File Name | Purpose | ROS Nodes |
| ---- | ---- | ---- |
| space_station_gnc/src/thruster_matrix.cpp | Define thruster matrix info. | none |
| space_station_gnc/src/physics_motion.cpp | input thruster force and output attitude. | AttitudeDynamicsNode |

## Topics

| Topic Name | published by (node class) | meaning | unit |
| ---- | ---- | ---- | ---- |
| /gnc/attitude_LVLH | AttitudeDynamicsNode | attitude quaternion | N/A |
| /gnc/angvel_body | AttitudeDynamicsNode | angular velocity | ? |
| /gnc/cmg_del | AttitudeDynamicsNode | ? | ? |
| /gnc/pose_all | AttitudeDynamicsNode | ? | ? |
| /gnc/cmg_h | AttitudeDynamicsNode | ? | ? |
| /gnc/ang_acc | AttitudeDynamicsNode | ? | ? |
| /gnc/bias_thruster_cmd | ControlTorque | thruster force | N ? |
