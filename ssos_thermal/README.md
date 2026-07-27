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

> **Status:** `thermal_network_physics`/`thermal_network_ros` implemented
> and passing tests (`colcon test --packages-select ssos_thermal`).
> `space_station_thermal_control`'s equivalent executables keep running
> unmodified — this package doesn't touch it and isn't wired into
> `space_station.launch.py` yet.

## Nodes

| Node | Executable | Model | Highlights |
|------|-----------|-------|-----------|
| **Thermal Network** | `thermal_network_node` | Lumped-node conductive/radiative network, `LifecycleNode` | RK4 integration over node/link graph loaded from YAML, coolant-loop feedback via `/coolant_heat_transfer` action, edge-triggered overheat fault |
| **Sun Vector** | `sun_vector_node` | Low-precision solar ephemeris + body-frame rotation | Port of the legacy `sun_vector`, Bullet Physics dropped for [`math3d.hpp`](include/ssos_thermal/math3d.hpp) |
| **Solar Heat** | `solar_heat_node` | Per-panel absorbed solar power | Port of the legacy `array_absorptivity`, same Bullet-free swap |

Only these three are in scope — `space_station_thermal_control`'s
`cooling_server`, `radiator`, and `demand` executables stay where they are;
see [REFACTOR_PLAN.md](REFACTOR_PLAN.md) for why.

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

The thermal network node is an `rclcpp_lifecycle::LifecycleNode`. It
self-activates shortly after launch (no external lifecycle `ChangeState`
call needed), registers with the `system_manager` via
`/ssos/register_subsystem`, and publishes telemetry alongside heartbeats on
`/ssos/thermal/heartbeat` and faults on `/ssos/fault_event`.

## Parameters

Every physical parameter (`enable_failure`, `enable_cooling`,
`cooling_trigger_threshold`, `max_temp_threshold`, `cooling_rate`,
`thermal_update_dt`, `thermal_config_file`) is a ROS 2 parameter,
live-tunable at runtime — no hardcoded thresholds in C++. See
[docs/parameters.md](docs/parameters.md) for the full reference, including
`sun_vector_node`/`solar_heat_node`.

## Documentation

- [docs/commands.md](docs/commands.md) — build/launch/inspect/test/shutdown
  command reference (pixi-wrapped)
- [docs/architecture.md](docs/architecture.md) — layering, data flow,
  stepping model, lifecycle/autostart
- [docs/parameters.md](docs/parameters.md) — parameter reference for all
  three nodes
- [docs/fault_catalog.md](docs/fault_catalog.md) — the one fault type this
  package raises today, and how the edge-trigger works
- [REFACTOR_PLAN.md](REFACTOR_PLAN.md) — the physics/ROS split and
  `LifecycleNode` migration this package implements, step by step, plus the
  Bullet Physics removal (`sun_vector`, `solar_heat_node`) in favor of small
  std-only `Vector3`/`Quaternion` structs
- [ECLSS_PATTERN_REFERENCE.md](ECLSS_PATTERN_REFERENCE.md) — the
  `ssos_eclss` structure this package mirrors
