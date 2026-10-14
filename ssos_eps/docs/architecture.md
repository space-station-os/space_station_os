# Architecture

`ssos_eps` is built in the same strictly separated layers as `ssos_eclss`.

```
+-------------------------------------------------------------+
| Layer 3: ROS (src/nodes, src/main)                          |
|   EpsNode (LifecycleNode)                                   |
|   EpsParameterBridge   EpsDiagnostics                       |
+----------------------------+--------------------------------+
                             | (owns, drives step())
                             v
+-------------------------------------------------------------+
| Layer 2: subsystem physics (NO ROS)                         |
|   system/EpsSystem                                          |
|   solar/   battery/   bcdu/                                 |
+----------------------------+--------------------------------+
                             | (uses)
                             v
+-------------------------------------------------------------+
| Layer 1: common (NO ROS)                                    |
|   units, interpolation                                      |
+-------------------------------------------------------------+
```

`eps_physics` (Layers 1-2) links nothing from ROS; the `eps_orbit_validation`
standalone links `eps_physics` only. `eps_ros` (Layer 3) adds `rclcpp`,
`rclcpp_lifecycle`, `space_station_interfaces` and the message packages.

## Power path and step order

```
 /sim/world_state
 (flux, eclipse)
        |
        v
 SolarArrayModel --array W--> SsuModel --160 V, available W--+
                                                            |  split evenly
                                                            v  per channel
                    +--------------- channel c (x12) ---------------+
                    |  BcduModel  <-- channel load share            |
                    |     |  surplus -> charge (limits, eta_c)      |
                    |     |  deficit -> discharge (limits, eta_d)   |
                    |     v                                         |
                    |  BatteryOruModel x2  (shared by capability)   |
                    +-----------------------------------------------+
```

`EpsSystem::step(dt, input)` runs the stages in that fixed order every call,
so the whole generation-to-storage chain is solved within one timestep with no
inter-node message latency. One lifecycle node therefore owns the full chain;
per-subsystem telemetry is still published on separate topics.

## Sign conventions

- Battery terminal power and current: positive out of the battery (discharge),
  negative into it (charge). This matches `BCDUStatus.current_draw`.
- `sensor_msgs/BatteryState.current` is published with the ROS convention
  (positive while charging), i.e. the negated model current.

## Health and faults

`EpsState` exposes `low_soc`, `load_unserved`, `bcdu_fault` and `oru_failed`;
`healthy` is false when any is set. The node sends a `FaultEvent` on the rising
edge of each condition (when `enable_auto_faults` is true) and re-arms once it
clears. The heartbeat carries `healthy` and the current status message.

The physics exposes fault hooks (`set_oru_failed`, `set_bcdu_fault`,
`set_array_degradation`). The node maps them to the parameters
`faults.failed_oru`, `faults.faulted_bcdu` and `solar.array.degradation_factor`
for what-if testing; routing faults through `ssos_sim` comes with the EPS
fault/scenario work.

## Stepping model

The node runs a wall timer at `step_rate_hz`. Each tick computes `dt` from ROS
time (so `/clock` from `ssos_sim` drives it, and frozen sim time gives
`dt = 0`), applies any pending live parameter changes, steps `EpsSystem`, then
publishes telemetry, faults and the heartbeat.
