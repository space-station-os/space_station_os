# ssos_eclss

High-fidelity, physics-based simulation of the International Space Station's
Environmental Control and Life Support System (ECLSS) for Space Station OS
(SSOS).

The defining architectural principle is a **strict separation between physics
and ROS**: everything under `src/{common,ars,ogs,wrs,sabatier,cabin,crew,faults}`
compiles with **zero ROS dependencies**, so the same physics can run in
simulation or on flight hardware. ROS lives only in `src/nodes/` and `src/main/`.

---

## 1. How the ISS ECLSS works

On the ISS, ECLSS is a set of coupled regenerative loops that keep the cabin
habitable from a nearly closed mass balance:

- The **crew** consume O₂ and water and produce CO₂, humidity (sweat +
  respiration), urine and waste.
- The **CDRA** (Carbon Dioxide Removal Assembly, a 4-Bed Molecular Sieve)
  scrubs CO₂ from cabin air; the desorbed CO₂ is vented or sent to Sabatier.
- The **Sabatier** reactor reacts that CO₂ with H₂ (`CO₂ + 4H₂ → CH₄ + 2H₂O`),
  recovering water and venting methane.
- The **WRS** (UPA + BPA + WPA) turns urine and humidity condensate back into
  potable water.
- The **OGS/OGA** electrolyses potable water into O₂ (to the cabin) and H₂ (to
  Sabatier).

The "water balance" the ISS flight-control team manages is exactly this: potable
water is consumed by the crew and by electrolysis, and replenished by recovery,
with the deficit made up by resupply.

## 2. How SSOS models it — the closed loop

Each subsystem is an independent lifecycle node. They are coupled purely through
`/ssos/*` topics, so the whole station-level mass balance emerges from the
individual physics models rather than being hard-coded:

```
                 crew CO2 ─────────────►┐
   ┌────────┐    crew O2  ◄──────┐      │      ┌──────────┐
   │  CREW  │    latent H2O ─────┼───┐  └─────►│  CABIN   │──ppCO2──┐
   │(diurnal│    urine ──────┐   │   │         │(atmosphere)         │
   │ sched) │    potable ◄─┐ │   │   │         └──────────┘         │
   └────────┘              │ │   │   │              ▲               │
                           │ │   │   │          O2  │               ▼
        ┌──────────────────┘ │   │   │              │          ┌─────────┐
        │        ┌───────────┘   │   │              │          │   ARS   │
        ▼        ▼               ▼   │        ┌──────┴───┐      │ (4BMS)  │
   ┌──────────────────┐          │   └───────►│   OGS    │      └────┬────┘
   │       WRS        │──potable─┼──feedwater─│(electrol)│    CO2 removal
   │  UPA+BPA+WPA     │◄─────────┘            └────┬─────┘           │
   │  potable tank    │◄────Sabatier H2O           │ H2              │
   │  wastewater tank │                            ▼                 │
   └──────────────────┘                       ┌─────────┐            │
                                               │SABATIER │◄──CO2──────┘
                                               │ CO2+4H2 │
                                               └─────────┘
```

- **Cabin**: crew CO₂ (source) − ARS removal (sink), OGS O₂ (source) − crew O₂
  (sink), plus leak. Publishes `/ssos/cabin/co2_ppm`.
- **ARS**: reads live cabin ppCO₂, scrubs proportionally, publishes removal.
- **Sabatier**: consumes ARS-desorbed CO₂ + OGS H₂, splits H₂-limited (react →
  water to WRS; remainder vented), holds catalyst temperature with a trim heater.
- **WRS**: collects urine into a wastewater tank, batch-processes it (UPA) plus
  humidity condensate + Sabatier water into a potable tank; serves crew drinking
  and OGS feedwater.
- **OGS**: draws feedwater from the WRS potable bus (feedwater-limited if empty),
  produces O₂ (cabin) and H₂ (Sabatier).

## 3. Subsystems

| Subsystem | Node | Model |
|-----------|------|-------|
| **Crew** | `crew_node` | Diurnal metabolic + water-stream generator (ICES-2021-365) |
| **Cabin** | `cabin_node` | Well-mixed O₂/CO₂/N₂/H₂O atmosphere + leak |
| **ARS** | `ars_node` | 4-Bed Molecular Sieve, 1D packed-bed PDE, 10-60-10 cycle |
| **OGS** | `ogs_node` | PEM electrolysis stack + gas separation |
| **WRS** | `wrs_node` | UPA (VCD) + BPA + WPA, wastewater/potable inventories |
| **Sabatier** | `sabatier_node` | `CO₂ + 4H₂ → CH₄ + 2H₂O`, thermostatic reactor |

Per-subsystem detail: [docs/ARS.md](docs/ARS.md), [docs/OGS.md](docs/OGS.md),
[docs/WRS.md](docs/WRS.md).

## 4. ROS 2 interface map

All nodes are `rclcpp_lifecycle::LifecycleNode`s that self-activate
(`autostart`/`autostart_delay_ms`), register with the `system_manager` via
`/ssos/register_subsystem`, and emit a heartbeat on `/ssos/<name>/heartbeat`.

| Node | Publishes | Subscribes |
|------|-----------|------------|
| `crew_node` | `/ssos/crew/co2_kg_day`, `/ssos/crew/o2_consumption_kg_day`, `/ssos/crew/latent_water_kg_day`, `/ssos/crew/urine_kg_day`, `/ssos/crew/potable_demand_kg_day`, `/ssos/crew/diagnostics` | — |
| `cabin_node` | `/ssos/cabin/co2_ppm`, `/ssos/cabin/diagnostics` | `/ssos/crew/co2_kg_day`, `/ssos/crew/o2_consumption_kg_day`, `/ssos/ars/co2_removal_kg_day`, `/ssos/ogs/o2_kg_day` |
| `ars_node` | `/ssos/ars/co2_removal_kg_day`, `/ssos/ars/bed_states`, `/ssos/ars/cycle_phase`, `/ssos/ars/diagnostics` | `/ssos/cabin/co2_ppm`, `/sim/world_state` |
| `sabatier_node` | `/ssos/sabatier/water_kg_day`, `/ssos/sabatier/diagnostics` | `/ssos/ars/co2_removal_kg_day`, `/ssos/ogs/o2_kg_day` |
| `wrs_node` | `/ssos/wrs/potable_kg_day`, `/ssos/wrs/potable_available_kg`, `/ssos/wrs/wastewater_kg`, `/ssos/wrs/diagnostics` | `/ssos/crew/urine_kg_day`, `/ssos/crew/latent_water_kg_day`, `/ssos/crew/potable_demand_kg_day`, `/ssos/ogs/water_demand_kg_day`, `/ssos/sabatier/water_kg_day` |
| `ogs_node` | `/ssos/ogs/o2_kg_day`, `/ssos/ogs/water_demand_kg_day`, `/ssos/ogs/diagnostics` | `/ssos/wrs/potable_available_kg` |

Common to all: publish `/ssos/<name>/heartbeat` (`SubsystemHeartbeat`) and
`/ssos/fault_event` (`FaultEvent`); client of `/ssos/register_subsystem`.
Scalar telemetry is `std_msgs/Float64`; `bed_states`/`cycle_phase` are
`std_msgs/Float64MultiArray`; `diagnostics` are `diagnostic_msgs/DiagnosticArray`.

**Services / actions.** The ECLSS nodes expose the standard lifecycle and
parameter services (`/<node>/get_parameters`, `/set_parameters`,
`/list_parameters`, `/<node>/change_state`, …). Fault injection is a service on
the simulation controller, `/sim/inject_fault` (see §7). There are no custom
actions in this package.

## 5. Diurnal simulation lifecycle (24 h)

The crew node drives a **diurnal activity schedule** over a configurable "day"
(`day_length_s`, default 86400 s = realtime). Metabolic **gas** rates follow the
ICES-2021-365 long-exercise profile; **water** streams are steady daily averages
(ISS water-balance paper). Position in the day is `sim_time / day_length_s`, so
the schedule runs on **sim time** and is sped up by the simulator's `time_scale`.

| Phase | Duration | CO₂ (g/min·person) | O₂ (g/min·person) | Latent H₂O (g/min·person) | Heat (W/person) |
|-------|----------|--------------------|-------------------|---------------------------|-----------------|
| sleep | 8.0 h | 0.44 | 0.37 | 1.08 | 88 |
| nominal | 6.0 h | 0.69 | 0.59 | 1.33 | 139 |
| exercise | 0.5 h | 5.22 | 3.99 | 10.0 | 968 |
| recovery | 4.0 h | 0.69 | 0.59 | 2.00 | 139 |
| nominal | 5.5 h | 0.69 | 0.59 | 1.33 | 139 |

The day starts at the beginning of the sleep block. Per-crew daily totals
reproduce the paper (~1.0 kg CO₂, ~0.85 kg O₂, ~2.2 kg latent water); for 4 crew
the cabin sees ~4.0 kg/day CO₂ and the ARS scrubs to match, so **CO₂ removal
rises and falls with activity** (lowest during sleep ~2.5 kg/day, spiking during
exercise) — this is the loop tracking load, not a fault.

**Speeding it up.** Two independent knobs:
- `time_scale` on `simulation_controller` — accelerates *all* sim time (set live
  from the GUI). One 24 h day at 60× = 24 min wall; sleep alone = 8 min wall.
- `day_length_s` on `crew_node` — compresses just the activity cycle (daily
  averages preserved). E.g. `day_length_s:=600` makes a full sleep→exercise
  cycle repeat every 600 s of sim time, so at 60× you watch it cycle every 10 s.

> Note: the cabin↔ARS CO₂ feedback is integrated with a small step cap for
> stability, so at very high `time_scale` the cabin gas loop advances more slowly
> than the water inventories. Both remain stable.

## 6. Water balance & UPA processing

The WRS maintains two inventories (published live, shown in the GUI):

- **Wastewater tank** (`/ssos/wrs/wastewater_kg`, WSTA, default cap 22 kg) —
  fills with crew urine + flush (~6 kg/day for 4 crew).
- **Potable tank** (`/ssos/wrs/potable_available_kg`, default cap 1000 kg, start
  300 kg) — filled by recovered water, drained by crew drinking and OGS feedwater.

**UPA batch cycle** (mirrors the ISS WSTA 70 % trigger):
1. Urine accumulates in the wastewater tank.
2. When it reaches `upa_start_fraction` (70 % → 15.4 kg), the UPA starts.
3. It processes at `max_urine_process_kg_day` (default 13.6 kg/day), recovering
   ~85 % as distillate to the potable tank, until the tank drains to
   `upa_stop_fraction` (5 %).

At the 4-crew load this is roughly a **~2.4-day fill** then a **~2-day
drain** per batch in realtime (much faster under `time_scale`). `days_of_supply`
= potable inventory ÷ daily potable demand is published in
`/ssos/wrs/diagnostics`.

## 7. Faults

Faults are **off by default** — a nominal run never trips. There are three ways
to introduce them:

**(a) Node auto-fault thresholds** (`enable_auto_faults`, default `false`).
Each of `cabin`, `ars`, `ogs`, `wrs` has a health threshold; enable the flag and
push the relevant limit to cross it. Example — trip the ARS:

```bash
ros2 param set /ars_node enable_auto_faults true
ros2 param set /ars_node co2_required_kg_day 999.0   # forces removal < required
```

The node then publishes a `FaultEvent` on `/ssos/fault_event` and its heartbeat
goes unhealthy (`system_manager` → DEGRADED). Node thresholds:
`cabin` co2 > `co2_alarm_ppm`; `ars` removal < `co2_required_kg_day`;
`ogs` O₂ < `o2_required_kg_day`; `wrs` conductivity > `potable_limit_us` or MF
breakthrough.

**(b) Injected fault via the simulation controller** — service
`/sim/inject_fault` (`space_station_interfaces/srv/InjectFault`):

```
target_subsystem  string   # e.g. "ars", "eclss"
fault_type        string   # e.g. "sensor_stuck_at"
parameters_json   string   # e.g. '{"stuck_value": 0.0}'
duration_s        float64
---
success  bool
message  string
```

```bash
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  '{target_subsystem: "eclss", fault_type: "sensor_stuck_at",
    parameters_json: "{\"stuck_value\": 0.0}", duration_s: 300.0}'
```

This publishes a `FaultEvent` for the `system_manager` and perturbs the world
model where wired.

**(c) Physics-level fault catalog** — `config/fault_definitions.yaml` defines
`faults::FaultDefinition` entries (heater/blower/precooler degradation, sensor
drift/bias/stuck, cabin leak, …) that alter the actual component physics. Format:

```yaml
faults:
  ars_heater_partial:
    type: heater_partial      # see docs/fault_catalog.md for types + magnitude meaning
    severity: warning         # warning | critical | emergency
    target: ars_heater
    magnitude: 0.368          # meaning is per-type (here: surviving power fraction)
    start_time_s: 600.0
    duration_s: -1.0          # < 0 = permanent
    description: "ARS regeneration heater partial element failure"
```

See [docs/fault_catalog.md](docs/fault_catalog.md) for every fault type and the
meaning of `magnitude`. `ros2 launch ssos_eclss eclss_with_faults.launch.py`
starts the full suite with the fault-definitions file exposed for scheduling via
`/sim/inject_fault`.

## 8. Parameters (tunable, live)

Every physical constant is a ROS 2 parameter. Node-level operating parameters
(crew rates, stack current, tank capacities, thresholds, `enable_auto_faults`)
apply **live** through each node's `on_set_parameters` handler; the ARS's full
physics set is mapped and validated through the single `EclssParameterBridge`
(static geometry needs a reconfigure; the rest is live). The mission-control GUI
exposes all of them through its **PARAMS** editor. See
[docs/parameters.md](docs/parameters.md).

## 9. Build, run, test

```bash
cd ~/ssos_ws
colcon build --packages-select ssos_eclss
source install/setup.bash

# Full ECLSS suite (crew + cabin + ARS + OGS + WRS + Sabatier), self-activating
ros2 launch ssos_eclss eclss.launch.py

# Accelerated time needs a clock: run the simulation controller and set time_scale
ros2 run ssos_sim simulation_controller --ros-args -p time_scale:=60.0
ros2 lifecycle set /simulation_controller configure
ros2 lifecycle set /simulation_controller activate

# Watch the loop
ros2 topic echo /ssos/cabin/co2_ppm
ros2 topic echo /ssos/wrs/potable_available_kg
ros2 topic echo /ssos/crew/diagnostics

colcon test --packages-select ssos_eclss && colcon test-result --verbose
```

Standalone (no ROS): `ars_validation`, `parameter_sweep`, `breakthrough_curve_gen`.

## 10. References

- **Crew:** Ewert, Downs, Keener, "Developing a Daily Metabolic Rate Profile for
  Human Exploration Missions", ICES-2021-365 (2021).
- **Water balance:** Tobias, Garr, Erne, "International Space Station Water
  Balance Operations" (AIAA).
- **ARS:** Peters, Cmarik, Knox, "4BCO2 EDU Performance", ICES-2021-313 (2021);
  "Integrated Evaluation of Closed-Loop Air Revitalization".
- **OGS:** Takada et al., "Status of the Advanced Oxygen Generation Assembly",
  ICES-2023-311 (2023).
- **WRS:** Williamson, Wilson, Robinson, Luong, "Status of ISS Water Management
  and Recovery", ICES-2023-097 (2023).
- **Sabatier:** Hintze et al., "Sabatier System Design Study for a Mars ISRU
  Propellant Production Plant", ICES-2018-155 (2018).
