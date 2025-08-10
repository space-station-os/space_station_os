# space_station_gnc

## Files Specification
| File Name | Main Purpose | ROS Nodes |
| ---- | ---- | ---- |
| space_station_gnc/src/thruster_matrix.cpp | Define thruster matrix info. | none |
| space_station_gnc/src/physics_motion.cpp | input thruster force and output attitude. | AttitudeDynamicsNode |
| space_station_gnc/src/orbit_dynamics.cpp | subscribe attitude and publish position at ECI. | OrbitDynamicsNode |

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

## ROS Nodes

### AttitudeDynamicsNode

Initial parameters:
- `timing.torque_dt`: simulation timestep [s]
- `timing.pub_dt`: publish timestep [s]
- `timing.publish_every`: ?
- `initial.attitude`: atitude quaternion [N/A]
- `initial.angvel`: angular velocity [rad/s?]
- `initial.angacc`: angluar acceleration [rad/s^2]
- `dynamics.total_mu`: constraint mu [?]

Principal functions:
- simulate attitude dynamics
  - Subscribe thruster torque
  - Publish attitude
  - simulate attitude dynamics
    - by RK4
    - thruster force and gravity gradient are considered

### OrbitDynamicsNode

Initial parameters:
- `timing.torque_dt`: simulation timestep [s]
- `timing.pub_dt`: publish timestep [s]
- `timing.publish_every`: ?
- `initial.tle_line2`: TLE line2. Position and velocity are calculated by it.
- `dynamics.total_mass`: total mass [kg]
- `dynamics.total_mu`: constraint mu [?]

Principal functions:
- simulate orbit dynamics
  - Subscribe thruster torque
  - Publish attitude
  - initial values are given by line #2 of TLE (two line element) format
  - simulate attitude dynamics
    - by Euler method
    - thruster force and gravity are considered
