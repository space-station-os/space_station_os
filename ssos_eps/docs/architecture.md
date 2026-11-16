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
|   solar/   battery/   bcdu/   mbsu/   ddcu/   loads/        |
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
                                                            |  shared evenly
                                                            v  per channel
           +----------------- channel c (x12) -----------------+
           |  BcduModel  <-- load routed by its MBSU            |
           |     |  surplus -> charge, deficit -> discharge     |
           |  BatteryOruModel x2                                |
           +--------------------------+-------------------------+
                                      | 3 channels per MBSU
                                      v
           +----------------- MbsuModel (x4) -------------------+
           |  usable sources only; demand shared by capability  |
           +--------------------------+-------------------------+
                                      | 2 DDCUs per MBSU
                                      v
           +----------------- DdcuModel (x8) -------------------+
           |  115-173 V in, 124.5 V out, 6.25 kW, losses, temp  |
           +--------------------------+-------------------------+
                                      |
                                      v
                 LoadRegistry: named loads, priority, DDCU
```

Each `EpsSystem::step(dt, input)`:

1. steps the array and SSU;
2. asks every channel's BCDU what it could deliver this step (array share plus
   battery discharge, within the BCDU limits) and at what bus voltage;
3. lets each MBSU pick its usable source channels;
4. sums the loads on each DDCU and sheds the least critical ones until the DDCU
   is within its rating, then sheds further across an MBSU's DDCUs until its
   sources can cover the DDCU input power;
5. routes each MBSU's demand onto its channels in proportion to their
   capability and steps the BCDUs and batteries with that load;
6. steps the DDCUs, browning out any whose MBSU delivered less than routed.

The whole generation-to-load chain is solved within one timestep with no
inter-node message latency. One lifecycle node therefore owns the full chain;
per-unit telemetry is still published on separate topics.

## Load admission

`request_load` checks the request is well formed, then picks the DDCU (or the
requested one) with the most rating headroom whose MBSU had enough spare supply
in the last step. Updating an existing load credits back its current power.
Loads stay registered while shed and come back on as soon as there is room.

## Sign conventions

- Battery terminal power and current: positive out of the battery (discharge),
  negative into it (charge). This matches `BCDUStatus.current_draw`.
- `sensor_msgs/BatteryState.current` is published with the ROS convention
  (positive while charging), i.e. the negated model current.

## Health and faults

`EpsState` exposes `low_soc`, `load_unserved`, `load_shed`, `bcdu_fault`,
`oru_failed`, `oru_overheat`, `ddcu_fault`, `ddcu_overload`, `mbsu_unstable` and
`redundancy_lost`; `healthy` is false when any is set. The node sends a `FaultEvent` on the rising
edge of each condition (when `enable_auto_faults` is true) and re-arms once it
clears. The heartbeat carries `healthy` and the current status message.

The physics exposes fault hooks (`set_oru_failed`, `set_bcdu_fault`,
`set_array_degradation`, `set_ddcu_fault`, `set_mbsu_fault`,
`set_channel_offline`). The node maps them to the parameters
`faults.failed_oru`, `faults.faulted_bcdu`, `faults.faulted_ddcu`,
`faults.faulted_mbsu`, `faults.offline_channel` and
`solar.array.degradation_factor` for what-if testing; routing faults through `ssos_sim` comes with the EPS
fault/scenario work.

## Stepping model

The node runs a wall timer at `step_rate_hz`. Each tick computes `dt` from ROS
time (so `/clock` from `ssos_sim` drives it, and frozen sim time gives
`dt = 0`), applies any pending live parameter changes, steps `EpsSystem`, then
publishes telemetry, faults and the heartbeat.
