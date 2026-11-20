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

To end a drill early, send `fault_type: clear_faults`; it is the only way to
clear one (a `clear` entry in the params is not recognised). The simulation
controller also rejects per-unit drills whose index is out of range, and EPS
ignores them as well. A drill belongs to EPS until it is cleared or expires:
it stays active across deactivate/activate, drills received while EPS is
inactive are recorded, and all are applied again on the first step after
activation. Only cleanup drops them. `system_manager` does not treat
`clear_faults` as a fault.

## Injectable faults

The station starts on orbit settled and healthy (bank at 95 %, every DDCU and
ORU at its thermal equilibrium) and stays NOMINAL until a drill is run. The
simulation controller runs one drill at a time.

| Type | Params (default) | Physical effect | Detected as | Recovery |
|------|------------------|-----------------|-------------|----------|
| `solar_array_degraded` | `factor` (0.3) | array output × factor | `solar_array_degraded` | output back to full when the drill ends |
| `array_gimbal_lock` | — | SARJ held gimbal-locked: array × `solar.array.gimbal_lock_derate` regardless of β | `solar_array_gimbal_lock` | tracking resumes when the drill ends |
| `battery_degraded` | `state_of_health` (0.5) | usable capacity of every ORU × state_of_health | `battery_degraded` when below `eps.min_state_of_health` | capacity restored when the drill ends |
| `battery_low_soc` | `soc` (0.15) | every ORU set to that charge when the drill starts | `low_battery_soc` while below `eps.low_soc_threshold` | the arrays recharge the bank; ending the drill does not change the charge |
| `battery_oru_failed` | `oru` (0) | ORU isolated, carries no current | `battery_oru_failed` | ORU back in service when the drill ends |
| `battery_oru_overheat` | `oru` (0), `heat_w` (800) | extra heat into that ORU's thermal node; at the default it passes `battery.max_temp_c` about 75 min after the drill starts (τ ≈ 2.8 h) | `battery_oru_overtemp` | the ORU cools below `battery.max_temp_c` once the drill ends |
| `bcdu_fault` | `channel` (0) | BCDU isolates its batteries; the array still feeds the channel | `bcdu_fault` | BCDU back when the drill ends |
| `mbsu_channel_loss` | `channel` (0) | feeder from the channel to its MBSU opens; demand is rerouted onto the other sources | `mbsu_feeder_open` (the MBSU keeps two sources) | feeder closes when the drill ends |
| `mbsu_fault` | `mbsu` (0) | MBSU bus isolated; its DDCUs lose primary power and their loads are shed | `mbsu_bus_unstable` | bus and loads back when the drill ends |
| `ddcu_fault` | `ddcu` (0) | DDCU trips; its loads are shed | `ddcu_fault` | DDCU and loads back when the drill ends |
| `ddcu_overheat` | `ddcu` (0), `heat_w` (400) | extra heat into that DDCU; it reaches `ddcu.trip_temp_c` in about 40 min, trips and sheds its loads; the extra heat alone holds it above `ddcu.reset_temp_c` | `ddcu_fault` | after the drill ends it cools below `ddcu.reset_temp_c` (about 15 min) and its loads come back |
| `load_spike` | `demand_scale` (2.0) | every load × demand_scale; loads over a DDCU rating are shed | `ddcu_overload` | demand back to nominal when the drill ends |
| `clear_faults` | — | ends the active drill | — | — |

Times are for the default parameters at 30 kW and the 18–20 °C cold plates;
use the simulation's `time_scale` to play the thermal drills out faster.

Injected faults combine with the what-if parameters (`faults.*`,
`solar.array.degradation_factor`, `battery.state_of_health`,
`loads.demand_scale`): a unit is faulted if either names it, and the factors
multiply.

## Detected faults

| `fault_type` | Severity | Condition |
|--------------|----------|-----------|
| `low_battery_soc` | CRITICAL | bank SOC below `eps.low_soc_threshold` |
| `load_unserved` | CRITICAL | requested load not fully delivered, and no DDCU overload, DDCU fault, unstable MBSU or low SOC already explains it |
| `bcdu_fault` | CRITICAL | a BCDU is faulted |
| `battery_oru_failed` | WARNING | an ORU is failed |
| `battery_oru_overtemp` | CRITICAL | an ORU is above `battery.max_temp_c` |
| `ddcu_fault` | CRITICAL | a DDCU carrying loads is tripped, or without primary power while its MBSU bus is up |
| `ddcu_overload` | WARNING | requested load on a DDCU exceeds its rating |
| `mbsu_bus_unstable` | CRITICAL | an MBSU has no usable source channel |
| `mbsu_redundancy_lost` | WARNING | an MBSU is down to one source channel |
| `mbsu_feeder_open` | WARNING | a channel's feeder to its MBSU is open |
| `solar_array_degraded` | WARNING | array output below beginning of life |
| `battery_degraded` | WARNING | battery state of health below `eps.min_state_of_health` |
| `solar_array_gimbal_lock` | WARNING | SARJ gimbal-locked; only the `array_gimbal_lock` drill does this with the default β limits |

Only the root cause is reported: a drill raises exactly one detected event
(the "Detected as" column), and loads it sheds are not reported again as
`load_unserved`. Each is published once when the condition starts (with
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
| `eps_channel_loss` | one channel feeder open (rerouted, `mbsu_feeder_open`) → recovery |
| `eps_low_battery_recovery` | bank drained to 15% in daylight → recharges past 20% → recovery |

```bash
ros2 launch ssos_eps eps_system.launch.py scenario:=eps_bcdu_fault_recovery
```
