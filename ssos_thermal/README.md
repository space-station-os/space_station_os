# ssos_thermal

Physics-based simulation of the space station's Active Thermal Control
System (ATCS) for Space Station OS (SSOS) v0.9.

This is a brand-new, parallel package to the existing
`space_station_thermal_control` (which it does not touch). Its defining
architectural principle, borrowed from `ssos_eclss`, is a **strict
separation between physics and ROS**: the thermal node/link network solver
under `src/network/` compiles with **zero ROS dependencies**, so the same
physics code can run in simulation or on flight hardware. ROS lives only in
`src/nodes/` and `src/main/`.

> **Status:** implemented, tested (`colcon test --packages-select
> ssos_thermal`), and started by `space_station.launch.py` through
> `space_station/launch/thermals.launch.py` with the station network
> (`config/station_thermal_nodes.yaml`). `space_station_thermal_control`'s
> equivalent executables keep running unmodified — this package doesn't
> touch it.

## Nodes

| Node | Executable | Model | Highlights |
|------|-----------|-------|-----------|
| **Thermal Network** | `thermal_network_node` | Lumped-node conductive network, `LifecycleNode` | RK4 integration of a node/link graph loaded from YAML, stepped on ROS time (sim time when `use_sim_time` is set), fixed-temperature boundary nodes, external heat loads from other subsystems on `/thermal/heat_loads`, per-node temperature limits with an edge-triggered overheat fault, simulation drills from `/ssos/fault_event`, coolant-loop feedback via `/coolant_heat_transfer` action |
| **Coolant** | `coolant_node` | Internal-loop-to-ammonia cooldown model, `LifecycleNode` | Serves the `/coolant_heat_transfer` action `thermal_network` uses to command cooling; publishes read-only `/thermal/coolant/status` for the mission-control GUI's coolant cards; best-effort vent via the legacy `radiator`'s `VentHeat` service |

Only these two are in scope — `space_station_thermal_control`'s
`radiator`, `demand`, `sun_vector`, and `array_absorptivity` executables
stay where they are; see [REFACTOR_PLAN.md](REFACTOR_PLAN.md) for why, for
what was intentionally left behind when `cooling_server` was ported to
`coolant_node` (an inert Behavior-Tree loop, dead publishers, unused
service clients), and for why `ssos_thermal`'s own Bullet-free ports of
`sun_vector`/`array_absorptivity` were later removed again.

## Build & test

```bash
cd ~/ssos_ws
colcon build --packages-select ssos_thermal
colcon test --packages-select ssos_thermal
colcon test-result --verbose
```

## Run the ROS nodes

```bash
ros2 launch ssos_thermal thermal.launch.py
```

`thermal_network` and `coolant_node` are `rclcpp_lifecycle::LifecycleNode`s.
Each self-activates shortly after launch (no external lifecycle
`ChangeState` call needed), publishes heartbeats on `/ssos/<name>/heartbeat`
and faults on `/ssos/fault_event`, and registers with the `system_manager`
via `/ssos/register_subsystem` (as `"thermal"` and `"coolant"` respectively —
both roll up into the mission-control GUI's single "Thermal" roster row).
Registration never blocks: after each heartbeat a node sends a request if the
service is up and none is pending (a request unanswered for 5 s is dropped),
and keeps trying until one is accepted, so
starting before `system_manager` is fine.

## Networks

| File | Nodes | Use |
|------|-------|-----|
| `config/thermal_nodes.yaml` | `base_link`, `SolarPanel1`, `SolarPanel2` | Standalone default: a 3-node star with no heat sink |
| `config/station_thermal_nodes.yaml` | `itcs_sink` (fixed 17 °C), `base_link` (cabin and structure), `itcs_coldplate` (EPS electronics cold plate), `SolarPanel1`, `SolarPanel2` | Full station: the internal coolant loop is the boundary node, so heat from EPS settles to a steady temperature instead of rising without bound |

A node with `fixed_temperature_c` is a boundary: it holds its temperature and
is left out of the average and hottest-node reports. Optional per-node keys:

- `initial_temperature_c`: start temperature (default 20 °C). Startup is
  deterministic; there is no random spread.
- `max_temperature_c`: the node's limit. Nodes without one use the
  `max_temp_threshold` parameter.

The station network starts at the steady state for the nominal EPS loads
(about 4.7 kW on `itcs_coldplate`, 26.7 kW delivered into `base_link`), so it
is settled and NOMINAL from the first step:

| Node | Initial | Limit | Why |
|------|---------|-------|-----|
| `itcs_sink` | 17 °C (fixed) | — | ITCS moderate-temperature loop supply |
| `base_link` | 21.68 °C | 27 °C | 17 + (1380 + 26700 + 2 × 10) / 6000; cabin comfort limit. Read as the cabin air temperature |
| `itcs_coldplate` | 21.70 °C | 28 °C | 17 + 4700 / 1000; below the EPS hardware limits (battery ORUs trip at 35 °C about 1 °C above the plate, DDCUs at 70 °C about 34 K above it), so thermal reports a hot plate before EPS hardware trips |
| `SolarPanel1/2` | 41.68 °C | 80 °C | base_link + 10 / 0.5; hot-case limit of silicon array blankets |

`base_link` lumps cabin air (about 0.47 MJ/K for ~390 m³), the pressure shell,
racks and equipment into 10 MJ/K, so the cabin has a 28 min time constant
and moves slowly, like the real one. The steady state depends only on the
conductances, so the capacity does not change the initial temperatures.
Crew metabolic heat (source `"crew"` on `base_link`) moves the cabin by about
0.02 °C per 100 W, well inside the margin. If EPS heat loads are missing for
a while (startup, a long pause), the cabin drifts towards 17.2 °C by about
0.16 °C per minute and recovers once they come back.

## Health and drills

`thermal_network` reports unhealthy while any non-boundary node is above its
limit and clears once every node is 1 °C back under it. One
`thermal_node_overheat` `FaultEvent` is published on the transition, not on
every step.

The simulation (`/sim/inject_fault`, scenario files) runs one drill at a time
on `/ssos/fault_event` with `subsystem_name: "thermal"`. The node parses it
with `ssos_sim::parse_injected_fault`, keeps it in an `ssos_sim::FaultTracker`
timed on ROS time, and restores the configuration when it expires or a
`clear_faults` arrives:

| Drill | Params | Effect |
|-------|--------|--------|
| `itcs_loop_degraded` | `sink_temp_c` (26) | Fixed sink held at `sink_temp_c` |
| `coldplate_conductance_loss` | `factor` (0.35) | `itcs_coldplate` link conductance × `factor` |

A drill outlives deactivation: one received while inactive is recorded, and
the active drill's effect is applied again on every activation.

See [docs/fault_catalog.md](docs/fault_catalog.md) for what each one does to
the station.

## Simulation time

Each tick integrates the ROS time elapsed since the last one (sim time when
`use_sim_time` is set) in RK4 sub-steps of at most `thermal_update_dt`. A
paused simulation does no physics and does not expire drills, but telemetry
and heartbeats keep publishing on the wall-clock timer.

## Heat loads from other subsystems

`thermal_network` subscribes to `/thermal/heat_loads`
(`space_station_interfaces/HeatLoadArray`: a `source` name and a list of
`{node_name, power}`). The latest set from each source is added to the named
nodes' internal power; a source that has not published for
`heat_load_timeout_s` is dropped. Published `ThermalNodeData.internal_power`
is the node's own power plus these loads. `ssos_eps` publishes its conversion
losses onto `itcs_coldplate` and the power delivered to loads onto
`base_link`, and reads the cold-plate temperature back from
`/thermal/nodes/state`.

## Parameters

Every physical parameter (`enable_cooling`,
`cooling_trigger_threshold`, `max_temp_threshold`, `cooling_rate`,
`thermal_update_dt`, `thermal_config_file`, plus `coolant_node`'s
`mass_kg`/`specific_heat_j_per_kg_c`/`heat_transfer_efficiency`/
`vent_threshold_kj`/`target_temp_c`) is a ROS 2 parameter, live-tunable at
runtime — no hardcoded thresholds in C++. See
[docs/parameters.md](docs/parameters.md) for the full reference.

## Documentation

- [docs/commands.md](docs/commands.md) — build/launch/inspect/test/shutdown
  command reference (pixi-wrapped)
- [docs/architecture.md](docs/architecture.md) — layering, data flow,
  stepping model, lifecycle/autostart
- [docs/parameters.md](docs/parameters.md) — parameter reference for both
  nodes
- [docs/fault_catalog.md](docs/fault_catalog.md) — the over-temperature
  fault this package raises and the drills it accepts
- [REFACTOR_PLAN.md](REFACTOR_PLAN.md) — the physics/ROS split and
  `LifecycleNode` migration this package implements, step by step, plus
  the Bullet-free `sun_vector_node`/`solar_heat_node` port and its later
  full removal as out of scope for this package
- [ECLSS_PATTERN_REFERENCE.md](ECLSS_PATTERN_REFERENCE.md) — the
  `ssos_eclss` structure this package mirrors
