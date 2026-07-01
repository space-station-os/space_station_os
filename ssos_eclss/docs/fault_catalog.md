# Fault catalog

Faults come in two layers:

- **Operational faults** — driven from ROS: a node's built-in health threshold
  (`enable_auto_faults`) and/or live parameter degradation. These act on the
  running nodes today and are what the GUI **FAULT** button uses.
- **Physics-level faults** — the ROS-free `faults::FaultInjector` taxonomy
  (`src/faults/`) catalogued in `config/fault_definitions.yaml`. These describe
  component-level perturbations (heater/blower/precooler degradation, sensor
  drift, cabin leak). `FaultSeverity` integers mirror
  `space_station_interfaces/msg/FaultEvent`.

> Wiring status: the operational paths (thresholds + parameter degradation)
> produce live effects now. The physics-level `FaultInjector` types are defined,
> unit-tested and catalogued, and `/sim/inject_fault` announces them on
> `/ssos/fault_event`; applying their component effects into the live ROS nodes
> is the current extension point (except `sensor_stuck_at` on target `eclss`,
> which perturbs the simulator's world O₂).

---

## 1. How a fault is triggered

**(a) GUI FAULT button** — opens the fault dialog, pick a fault, *Inject Fault*;
*Restore Nominal* reverts. Each entry runs a `/sim/inject_fault` announcement
plus the live parameter recipe below.

**(b) ROS CLI** — the same recipes by hand:

```bash
# announce (publishes a FaultEvent to the system manager + event feed)
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  '{target_subsystem: "ogs", fault_type: "ogs_generation_failure",
    parameters_json: "{}", duration_s: -1.0}'

# produce the physical effect via a live parameter
ros2 param set /ogs_node stack_current_a 0.0
```

**(c) `fault_definitions.yaml`** — physics-level catalog scheduled by the
simulation controller (format in §4).

Enabling a subsystem's threshold alarm:

```bash
ros2 param set /<node> enable_auto_faults true   # off by default
```

---

## 2. Per-subsystem fault reference

### ARS — Air Revitalization

| Fault | Trigger | Effect |
|-------|---------|--------|
| CO₂ scrubber degraded | `ars.efficiency.capture_efficiency` → 0.2 | removal collapses, cabin CO₂ rises |
| Regen heater failure | `ars.heater.total_power_w` → 0 | beds don't desorb; capacity fades over cycles |
| Blower degraded | `ars.operating.inlet_flow_scfm` → low | reduced process flow → less removal |
| Low-removal alarm | `enable_auto_faults:=true`, removal < `co2_required_kg_day` (4.16) | `co2_removal_below_requirement` CRITICAL + unhealthy |
| Heater/blower/precooler degradation | catalog `ars_heater_partial`, `ars_blower_degraded`, `ars_precooler_degraded` | component-level (see §3) |

### OGS — Oxygen Generation

| Fault | Trigger | Effect |
|-------|---------|--------|
| O₂ generation failure | `stack_current_a` → 0 | O₂ production stops; cabin O₂ balance negative |
| Feedwater starvation | (emergent) WRS potable tank empty | `feedwater_limited`, O₂ output falls |
| Low-O₂ alarm | `enable_auto_faults:=true`, O₂ < `o2_required_kg_day` (2.3) | `o2_production_low` CRITICAL + unhealthy |

### WRS — Water Recovery

| Fault | Trigger | Effect |
|-------|---------|--------|
| UPA assembly failure | `max_urine_process_kg_day` → 0 | urine unprocessed; wastewater tank fills, potable declines |
| Water quality out of spec | `enable_auto_faults:=true`, conductivity > `potable_limit_us` (100) or MF breakthrough | `water_quality_out_of_spec` CRITICAL + unhealthy |

### Cabin / sensors

| Fault | Trigger | Effect |
|-------|---------|--------|
| Cabin CO₂ alarm | `enable_auto_faults:=true`, ppCO₂ > `co2_alarm_ppm` (7000) | `co2_high` CRITICAL + unhealthy |
| Micrometeoroid leak | catalog `cabin_micrometeoroid_leak` | extra leak to vacuum |
| O₂ sensor stuck | `/sim/inject_fault` `sensor_stuck_at` target `eclss` | announced; perturbs world O₂ |
| CO₂ sensor drift | catalog `co2_sensor_drift` | reported reading drifts (§3) |

---

## 3. Physics-level fault types

Component perturbations tracked by `faults::FaultInjector`. `magnitude` meaning
is per-type:

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

`warning` (0), `critical` (1), `emergency` (2) — matching the `FaultEvent`
constants.

---

## 4. `fault_definitions.yaml` format

Each entry maps to a `faults::FaultDefinition`. `duration_s < 0` is permanent.
The `FaultInjector` registry tracks each fault's active window
(`start_time_s`, `duration_s`), aggregates effects per target component, and
applies sensor transforms; component models query the aggregated effect each
step.

```yaml
faults:
  ars_heater_partial:
    type: heater_partial      # see §3 for magnitude meaning
    severity: warning         # warning | critical | emergency
    target: ars_heater
    magnitude: 0.368          # 7 of 19 elements survive
    start_time_s: 600.0
    duration_s: -1.0          # permanent
    description: "ARS regeneration heater partial element failure"
```

Current catalog entries: `ars_heater_partial`, `ars_blower_degraded`,
`ars_precooler_degraded`, `co2_sensor_drift`, `cabin_micrometeoroid_leak`.
`ros2 launch ssos_eclss eclss_with_faults.launch.py` exposes this file for
scheduling via `/sim/inject_fault`.
