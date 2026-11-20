# Fault catalog

## Raised by `thermal_network`

| Type | Severity | Trigger | `description` |
|------|----------|---------|----------------|
| `thermal_node_overheat` | `SEVERITY_CRITICAL` | A non-boundary node above its `max_temperature_c` (or `max_temp_threshold` if it has none) | `"overheating: <node_name>"` |

`affected_interfaces` is always `["/thermal/nodes/state"]`.

The heartbeat's `healthy` field is false while the node is over temperature.
It clears once every node is 1 °C below its limit, so a node sitting right
at the limit does not flap.

`CoolantNode` raises none: it always reports `healthy=true` on
`/ssos/coolant/heartbeat`. Its most likely real fault condition -- venting
triggered but the legacy `radiator`'s `VentHeat` service unavailable -- is
only logged (`RCLCPP_ERROR`) today, not surfaced as a `FaultEvent`.

### Edge-triggered, not level-triggered

`ThermalDiagnostics::should_raise_fault(healthy)` only returns true on the
healthy→unhealthy transition, so a single `FaultEvent` publishes when a node
first crosses its limit, not once per tick for as long as it stays hot.

## Drills accepted by `thermal_network`

The simulation injects drills on `/ssos/fault_event` with
`subsystem_name: "thermal"` and a description in the `ssos_sim` fault format
(`ssos_sim/fault_format.hpp`). Only one drill is active at a time; a new one
replaces it, and it ends when its `duration_s` of ROS time has passed
(`duration_s <= 0`: until cleared) or on `clear_faults`. Ending a drill puts
the configured sink temperature and conductances back. Unknown types are
logged and ignored.

Effects below are for the station network with the nominal EPS loads.
Every drill trips `itcs_coldplate`'s 28 °C limit (one `thermal_node_overheat`
fault) while the EPS hardware on the plate stays inside its own limits
(battery ORUs about 31.5 °C at most against a 35 °C trip, the busiest DDCU
about 68 °C against 70 °C), so a drill never cascades into an EPS fault. Trip times are checked
by `StationThermalNetwork.EveryDrillTripsALimit`.

A drill outlives deactivation: one received while the node is inactive is
recorded, and the active drill's effect is applied again on activation.

| Type | Params (default) | What it does | Expected result |
|------|------------------|--------------|-----------------|
| `itcs_loop_degraded` | `sink_temp_c` (26) | Holds the fixed sink (`itcs_sink`) at `sink_temp_c` | Nodes head 9 °C warmer. `itcs_coldplate` (time constant 50 s) heads for 30.7 °C and trips 28 °C about 1 min in (50 s × ln(9 / 2.7) ≈ 60 s); ORUs peak about 31.5 °C, the busiest DDCU about 68 °C. `base_link` (28 min) heads for 30.7 °C and would pass 27 °C after about 25 min (1667 s × ln(9 / 3.68) ≈ 1491 s) |
| `coldplate_conductance_loss` | `factor` (0.35) | Multiplies the conductance of every `itcs_coldplate` link by `factor` | Plate time constant grows from 50 s to 143 s; heading for 17 + 4700 / 350 = 30.4 °C, it trips 28 °C about 3 min in (143 s × ln(8.73 / 2.43) ≈ 183 s); ORUs peak about 31.4 °C, the busiest DDCU about 68 °C. A factor below about 0.30 (plate above 32.5 °C) would trip that DDCU too |

After a drill the plate falls back under 27 °C (limit minus the 1 °C
hysteresis) within about half a minute to two minutes and the heartbeat goes
healthy again.

```bash
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  "{target_subsystem: thermal, fault_type: itcs_loop_degraded, \
    parameters_json: '{\"sink_temp_c\": 26.0}', duration_s: 300.0}"
ros2 topic echo /ssos/fault_event        # one thermal_node_overheat at the transition
ros2 topic echo /ssos/thermal/heartbeat  # healthy=false from ~1 min in until the plate is back under 27 degC
```
