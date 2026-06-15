# Fault catalog

Faults are physics-level: they alter the actual model behaviour, not just
reported values (except sensor faults, which corrupt a reported reading only).
The taxonomy is ROS-free; `FaultSeverity` integer values mirror
`space_station_interfaces/msg/FaultEvent` so the ROS layer maps them directly.

## Fault types

| Type | Class | `magnitude` meaning | Effect |
|------|-------|---------------------|--------|
| `sensor_stuck` | sensor | latched value (0 = latch first reading) | reading frozen |
| `sensor_drift` | sensor | drift rate [units/s] | reading += rate·t |
| `sensor_bias` | sensor | offset | reading += offset |
| `sensor_noise` | sensor | std-dev | reading += N(0, σ) |
| `sensor_scale` | sensor | multiplier | reading ×= factor |
| `valve_stuck_open` | actuator | — | position forced to 1 |
| `valve_stuck_closed` | actuator | — | position 0, flow 0 |
| `blower_degraded` | actuator | residual effectiveness ∈ [0,1] | flow ×= factor |
| `blower_failed` | actuator | — | flow 0 |
| `pump_failed` | actuator | — | throughput 0 |
| `heater_partial` | thermal | surviving power fraction | heater power ×= factor |
| `heater_failed` | thermal | — | heater power 0 |
| `precooler_degraded` | thermal | residual UA fraction | precooler UA ×= factor |
| `insulation_loss` | thermal | wall-loss amplification ≥ 1 | wall HTC ×= factor |
| `cabin_leak` | environmental | equivalent orifice area [m²] | extra leak to vacuum |

## Severity

`warning` (0), `critical` (1), `emergency` (2) — matching the FaultEvent
constants.

## Injection

The `FaultInjector` registry tracks each fault's active window
(`start_time_s`, `duration_s`; `duration_s < 0` is permanent), aggregates
effects per target component, and applies sensor transforms. Component models
query the aggregated effect each step. Definitions are catalogued in
`config/fault_definitions.yaml` and can be scheduled by the simulation
controller via `/sim/inject_fault`.

## Example

```yaml
ars_heater_partial:
  type: heater_partial
  severity: warning
  target: ars_heater
  magnitude: 0.368      # 7 of 19 elements survive
  start_time_s: 600.0
  duration_s: -1.0
  description: "ARS regeneration heater partial element failure"
```
