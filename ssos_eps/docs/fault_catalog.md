# Fault catalog

EPS faults come from two directions:

- **Injected** — the simulation (`/sim/inject_fault` or a scenario's fault
  schedule) publishes a `FaultEvent` for subsystem `eps`. `eps_node` applies it
  to the physics through the fault hooks.
- **Detected** — the physics state crosses a health condition and `eps_node`
  publishes a `FaultEvent` of its own on `/ssos/fault_event`, once per onset.

`system_manager` sees both. When the injected fault ends and EPS reports
healthy again for `recovery_hold_s`, it clears the fault and goes
DEGRADED → RECOVERY → NOMINAL.

## Injecting a fault

Through the service:

```bash
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  "{target_subsystem: eps, fault_type: bcdu_fault, parameters_json: '{\"channel\": 3}', duration_s: 60.0}"
```

or as a scenario entry (`ssos_sim/scenarios/*.yaml`):

```yaml
- name: bcdu_channel_3
  type: bcdu_fault
  target_subsystem: eps
  trigger: {type: time, at_s: 60.0}
  params: {channel: 3}
  duration_s: 120
```

`duration_s` is in simulation seconds; 0 or negative keeps the fault until it
is cleared. EPS recognises simulation faults by the description the simulation
controller writes (`Injected fault: …` / `Scheduled fault: …`), so its own
detected-fault events are never re-applied.

To clear: `fault_type: clear_faults` removes every injected EPS fault, and any
type with `{"clear": true}` (plus its index, if it has one) removes just that
fault. `system_manager` does not treat `clear_faults` as a fault.

## Injectable faults

| Type | Params (default) | Physical effect | Detected as |
|------|------------------|-----------------|-------------|
| `solar_array_degraded` | `factor` (0.3) | array output × factor | `solar_array_degraded` |
| `battery_degraded` | `state_of_health` (0.5) | usable capacity of every ORU × state_of_health | `battery_degraded` when below `eps.min_state_of_health` |
| `battery_low_soc` | `soc` (0.15) | every ORU set to that charge (one-shot, nothing to clear) | `low_battery_soc` while below `eps.low_soc_threshold` |
| `battery_oru_failed` | `oru` (0) | ORU isolated, carries no current | `battery_oru_failed` |
| `bcdu_fault` | `channel` (0) | BCDU isolates its batteries; the array still feeds the channel | `bcdu_fault` |
| `mbsu_channel_loss` | `channel` (0) | feeder from the channel to its MBSU opens; demand is rerouted | `mbsu_redundancy_lost` when an MBSU is down to one source, `mbsu_bus_unstable` at none |
| `mbsu_fault` | `mbsu` (0) | MBSU bus isolated; its DDCUs lose primary power | `mbsu_bus_unstable`, `ddcu_fault` |
| `ddcu_fault` | `ddcu` (0) | DDCU trips; its loads lose power | `ddcu_fault` |
| `load_spike` | `demand_scale` (2.0) | every load × demand_scale | `ddcu_overload`, `load_unserved` when loads are shed |
| `clear_faults` | — | removes all injected EPS faults | — |

Injected faults combine with the what-if parameters (`faults.*`,
`solar.array.degradation_factor`, `battery.state_of_health`,
`loads.demand_scale`): a unit is faulted if either names it, and the factors
multiply.

## Detected faults

| `fault_type` | Severity | Condition |
|--------------|----------|-----------|
| `low_battery_soc` | CRITICAL | bank SOC below `eps.low_soc_threshold` |
| `load_unserved` | CRITICAL | requested load not fully delivered |
| `bcdu_fault` | CRITICAL | a BCDU is faulted |
| `battery_oru_failed` | WARNING | an ORU is failed |
| `battery_oru_overtemp` | CRITICAL | an ORU is above `battery.max_temp_c` |
| `ddcu_fault` | CRITICAL | a DDCU carrying loads is tripped or without primary power |
| `ddcu_overload` | WARNING | requested load on a DDCU exceeds its rating |
| `mbsu_bus_unstable` | CRITICAL | an MBSU has no usable source channel |
| `mbsu_redundancy_lost` | WARNING | an MBSU is down to one source channel |
| `solar_array_degraded` | WARNING | array output below beginning of life |
| `battery_degraded` | WARNING | battery state of health below `eps.min_state_of_health` |

Each is published once when the condition starts (with
`enable_auto_faults: true`) and re-arms when it clears. The heartbeat carries
`healthy: false` and the status message for as long as any condition holds.

## Environment

Sunlight and eclipse are not faults: EPS takes `solar_flux_w_m2` and
`in_eclipse` from `/sim/world_state` only. The scenario
`eps_eclipse_transition` starts the orbit just before eclipse entry
(`initial_conditions.orbit_phase: 0.6`) to show the BCDUs switching from
charging to discharging.

## Scenarios

| Scenario (`ssos_sim/scenarios`) | Shows |
|---------------------------------|-------|
| `eps_eclipse_transition` | sunlight → eclipse, BCDUs charging → discharging, no faults |
| `eps_bcdu_fault_recovery` | BCDU fault for 120 s → DEGRADED → RECOVERY → NOMINAL |
| `eps_load_spike` | demand × 2 for 120 s → DDCU overload and priority shedding → recovery |
| `eps_channel_loss` | one channel lost (rerouted, still healthy), then a second (redundancy lost) → recovery |
| `eps_low_battery_recovery` | bank drained to 15% in daylight → recharges past 20% → recovery |

```bash
ros2 launch ssos_eps eps_system.launch.py scenario:=eps_bcdu_fault_recovery
```
