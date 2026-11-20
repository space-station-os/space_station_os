# Fault catalog

## Drills

The station starts on orbit and healthy; faults happen only as drills, one at
a time. A drill comes from `/sim/inject_fault`, a scenario file or the GUI
fault injector; `ssos_sim` refuses a second drill while one is running and
publishes the drill on `/ssos/fault_event`. Each ECLSS node applies drills
addressed to its own subsystem name, reports the effect through its heartbeat
(unhealthy, with one `FaultEvent` naming the detected fault), and returns to
normal when the drill expires or `clear_faults` arrives.

| Target | Drill | Params (default) | Physical effect | Detected as | Recovery |
|--------|-------|------------------|-----------------|-------------|----------|
| `crew` | `crew_metabolic_surge` | `scale` (2.0) | Crew CO2, O2, humidity and heat loads multiplied | `crew_metabolic_surge` (warning) | Loads back to plan |
| `cabin` | `cabin_leak` | `area_m2` (3e-6, a 2 mm hole) | Choked-flow air loss, about 38 kg/day; N2 makeup draws on the tanks, OGS raises O2 | `cabin_leak` (leak rate above `leak_alarm_kg_day`) | Hole closed; tanks keep the gas they lost |
| `cabin` | `o2_sensor_stuck` | `value_pct` (23.0) | The O2 reading OGS controls on is stuck; true O2 drifts as OGS throttles | `o2_sensor_disagree` (warning, analyser vs sensor) | Sensor restored; OGS pulls O2 back |
| `cabin` | `chx_failure` | none | CHX stops condensing; humidity rises (above 75% in about 6 h), condensate to WRS stops | `humidity` (warning) | CHX resumes and dries the cabin |
| `ars` | `ars_scrubber_degraded` | `capture_efficiency` (0.2) | Sorbent capture efficiency reduced | `co2_removal_low` (capture efficiency averaged over about a half-cycle, within about an hour) | Parameters restored; the average recovers over about an hour |
| `ars` | `ars_blower_failure` | none | No process flow through the beds | `ars_no_process_flow` | Blower restarts |
| `ars` | `ars_heater_failure` | none | Bed heaters off; unregenerated beds saturate over about a half-cycle and system capture falls to 10% of nominal (a cycle-average model, see below) | `co2_removal_low` after about 2 h | Heaters back; capture recovers over the next half-cycles |
| `ogs` | `ogs_generation_failure` | none | Stack current 0; cabin ppO2 falls about 0.7 kPa/day | `o2_generation_low` (O2 output against the commanded current) | ppO2 controller restores the cabin |
| `wrs` | `upa_assembly_failure` | none | Urine processor stops; wastewater tank fills | `upa_assembly_failure` (the UPA's built-in test) | UPA resumes its batches |
| `wrs` | `wpa_multifiltration_breakthrough` | none | Bed exhausted; off-spec product held in the WPA waste tank, not stored as potable | `water_quality_out_of_spec` | A fresh bed is installed; held water is reprocessed |
| `sabatier` | `sabatier_reactor_failure` | none | No methanation; all ARS CO2 vented, no water to WRS | `sabatier_no_conversion` (warning, conversion against the feed) | Reactor converting again |

Faults the nodes detect on their own, with no drill: `co2_high`, `o2_low`,
`o2_high`, `cabin_pressure` and `humidity` (cabin limits in
`cabin_parameters.yaml`),
`ogs_feedwater_low`, `wastewater_tank_full`, `potable_water_low`,
`sabatier_reactor_cold`, and `<unit>_power_loss` when EPS sheds a unit.
None of them trips in a nominal run; a drill's consequences can trip them
(for example `co2_high` after a long ARS failure). Every detection comes from
the unit's measured performance, except the UPA, whose controller reports its
own failure as the flight unit does, and the crew surge, which is a planned
activity deviation rather than a failure.

The ARS system removal is the cycle-average capture (capture efficiency times
the CO2 drawn in), so the heater drill reduces it through a bed-saturation
fraction; the bed PDE itself (bed temperatures and loadings on the GUI) runs
alongside with the heaters off.

A unit that stops publishing is treated as delivering nothing (ARS removal,
OGS O2, Sabatier water, condensate fall to zero after 5 s), so its loss shows
in the physics as well as in its heartbeat timing out.

Example:

```bash
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  "{target_subsystem: cabin, fault_type: cabin_leak, parameters_json: '{\"area_m2\": 3e-6}', duration_s: 3600.0}"
```

## Component fault library

`src/faults` holds a ROS-free, component-level fault taxonomy used by the
physics tests. The station drills above are applied by the nodes directly.

### Fault types

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

### Severity

`warning` (0), `critical` (1), `emergency` (2) — matching the FaultEvent
constants.

### Injection

The `FaultInjector` registry tracks each fault's active window
(`start_time_s`, `duration_s`; `duration_s < 0` is permanent), aggregates
effects per target component, and applies sensor transforms. Component models
query the aggregated effect each step. Definitions are catalogued in
`config/fault_definitions.yaml` and can be scheduled by the simulation
controller via `/sim/inject_fault`.

### Example

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
