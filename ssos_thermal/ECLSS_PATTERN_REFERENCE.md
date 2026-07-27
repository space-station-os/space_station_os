# How `ssos_eclss` is structured (reference for the thermal refactor)

## TL;DR

`ssos_eclss` splits cleanly into **physics that knows nothing about ROS**
and **ROS nodes that are thin wrappers around that physics**. Everything
else in the package (CMake targets, tests, config, docs) mirrors that same
split. This doc maps out the pattern so `space_station_thermal_control` can
be reshaped the same way later.

```
ssos_eclss/
├── include/ssos_eclss/     headers, same sub-folders as src/
├── src/
│   ├── common/              Layer 1 — shared physics primitives   \
│   ├── ars/ ogs/ wrs/                                              |  NO ROS
│   │   sabatier/ cabin/     Layer 2 — one subsystem's physics      |  (eclss_physics lib)
│   │   faults/                                                    /
│   └── nodes/               Layer 3 — ROS wrapper per subsystem  --- eclss_ros lib
│   └── main/                one tiny main.cpp per executable
├── standalone/               ROS-free CLI tools, link eclss_physics only
├── config/                   one YAML per subsystem
├── launch/                   full + per-subsystem + fault-injection variants
├── test/
│   ├── unit/<subsystem>/    mirrors src/<subsystem>/
│   ├── integration/          multi-component physics tests
│   └── ros/                  node-level tests (lifecycle, params)
└── docs/                      architecture.md + one doc per subsystem
```

## The core idea: physics and ROS are different libraries

This is the one rule everything else follows. From `CMakeLists.txt`:

```cmake
# ---- Pure physics library (NO ROS) ----
add_library(eclss_physics
  src/common/...   src/ars/...   src/ogs/...   src/wrs/...
  src/sabatier/... src/cabin/... src/faults/...
)
# NOTE: eclss_physics links NOTHING from ROS

# ---- ROS node library ----
add_library(eclss_ros
  src/nodes/eclss_parameter_bridge.cpp
  src/nodes/eclss_diagnostics.cpp
  src/nodes/ars_node.cpp   src/nodes/ogs_node.cpp   ...
)
target_link_libraries(eclss_ros eclss_physics)
ament_target_dependencies(eclss_ros rclcpp rclcpp_lifecycle space_station_interfaces ...)

# ---- one executable per node, links eclss_ros ----
add_executable(ars_node src/main/ars_main.cpp)
target_link_libraries(ars_node eclss_ros)
```

**Why it matters (from the package README):** the same physics object
"can therefore be embedded in a flight controller or in this simulation
unchanged." The `standalone/` executables (`ars_validation`,
`parameter_sweep`, `breakthrough_curve_gen`) *prove* the split is real —
they link only `eclss_physics` and run with zero ROS, no `rclcpp::init`
anywhere.

```
 eclss_physics  <───────────┐
 (zero ROS)                 │ link only this
   ▲                        │
   │ link + wrap             standalone/*.cpp   (CLI validation, no ROS)
   │
 eclss_ros
 (rclcpp, rclcpp_lifecycle, space_station_interfaces)
   ▲
   │ link
 ars_node / ogs_node / wrs_node / cabin_node / sabatier_node  (executables)
```

## Anatomy of one node (they're all shaped the same)

Every subsystem node (`ArsNode`, `OgsNode`, `WrsNode`, `CabinNode`,
`SabatierNode`) is a `rclcpp_lifecycle::LifecycleNode` with the identical
skeleton — once you've read one (`ars_node.hpp`/`.cpp`), you've read them
all:

| Member group | Example | Purpose |
|---|---|---|
| Physics | `std::unique_ptr<ars::FourBedSystem> ars_;` | the ROS-free object this node drives |
| Parameter bridge | `std::unique_ptr<EclssParameterBridge> bridge_;` | maps ROS params ↔ physics struct |
| Publishers | `co2_removal_pub_`, `telemetry_pub_`, ... | subsystem-specific telemetry |
| **Standard trio** | `heartbeat_pub_`, `fault_pub_`, `register_client_` | the ssos_core contract — identical on every node |
| Subscriber | `world_state_sub_` | `/sim/world_state`, the Epic A boundary |
| Timers | `step_timer_`, `autostart_timer_` | drive `step()`; auto-configure+activate on launch |
| `was_healthy_` | bool | edge-trigger so faults publish once per transition, not every tick |

Lifecycle callbacks always do the same jobs, in the same order:

```
on_configure()  → build physics object from params, create all publishers/
                  subscriber/service client, add param-validation callback
on_activate()   → activate lifecycle publishers, start step_timer_,
                  call register_with_manager()   [see below]
on_deactivate() → deactivate publishers, cancel timer
on_cleanup()    → reset everything
step()          → tick physics, publish telemetry, check health,
                  publish heartbeat every tick, publish fault on edge only
```

## The ssos_core contract — the part that matters for the thermal refactor

This is the exact piece `space_station_thermal_control` is missing today.
Every node does three things, all visible in `ars_node.cpp`:

1. **Register once**, when activated (`on_activate()` → `register_with_manager()`):
   waits ≤200ms for `/ssos/register_subsystem`, warns-and-continues if
   `system_manager` isn't up yet (never blocks startup), otherwise sends
   `{subsystem_name, published_topics, subscribed_topics, heartbeat_topic}`.
2. **Heartbeat every tick** on `/ssos/<name>/heartbeat`
   (`SubsystemHeartbeat`: lifecycle state + `healthy` + a status string).
3. **Fault on the healthy→unhealthy edge only**, on `/ssos/fault_event`
   (`FaultEvent`: type, severity, description, affected interfaces) — guarded
   by a `was_healthy_` bool so one bad reading doesn't spam the fault bus.

`EclssDiagnostics` (`src/nodes/eclss_diagnostics.{hpp,cpp}`) is a small
static-method helper shared by all five nodes so the heartbeat/fault message
construction isn't copy-pasted five times: `make_heartbeat(...)`,
`make_fault(...)`, plus a couple of subsystem-agnostic health checks and
`maybe_autostart()` (a one-shot timer that self-configures+activates a node
shortly after launch, so a launch file doesn't need to fire lifecycle
`ChangeState` events for every managed node individually).

## Config, launch, tests: same mirroring principle

- **`config/`** — one YAML per subsystem (`ars_parameters.yaml`,
  `ogs_parameters.yaml`, ...), loaded through `EclssParameterBridge` so no
  physical constant is ever hardcoded in C++.
- **`launch/`** — `eclss.launch.py` (everything), `ars_only.launch.py` /
  `ogs_only.launch.py` / `wrs_only.launch.py` (one subsystem, for focused
  testing), `eclss_with_faults.launch.py` (fault-injection scenario). Same
  granularity space_station_thermal_control's `launch/thermals.launch.py`
  could split into.
- **`test/`** — directory shape mirrors `src/`: `test/unit/ars/`,
  `test/unit/cabin/`, ... one test file per physics component;
  `test/integration/` for multi-component/closed-loop tests;
  `test/ros/` for node-level tests (`test_ars_node.cpp`,
  `test_parameter_bridge.cpp` — lifecycle transitions, parameter validation).
- **`docs/`** — `architecture.md` (the layering diagram + data-flow, this
  doc's inspiration), then one physics+ROS reference doc per subsystem
  (`ARS.md`, `OGS.md`, `WRS.md`) citing the ISS engineering papers each
  model is validated against, plus `parameters.md`, `validation.md`,
  `fault_catalog.md`.

## If `space_station_thermal_control` followed this pattern

Not a commitment, just the shape it would take — for comparison against
what exists today:

| ssos_eclss | space_station_thermal_control today | equivalent under this pattern |
|---|---|---|
| `eclss_physics` (no ROS) | thermal physics is inline inside `ThermalSolverNode` (`thermals_solver.cpp`) | extract into a `thermal_physics` lib (RK4 node-network solver, no `rclcpp`) |
| `eclss_ros` + per-node executables | 5 flat executables (`cooling_server`, `radiator`, `thermal_nodes`, `demand`, `array_absorptivity`), all plain `rclcpp::Node` | a `thermal_ros` lib of thin node wrappers, one per executable |
| `EclssDiagnostics` shared helper | heartbeat/fault would be hand-written per node (see `REFACTOR_PLAN.md`) | a shared `ThermalDiagnostics` helper once more than one thermal node registers |
| lifecycle nodes (`on_configure`/`on_activate`) | plain `rclcpp::Node`, always "active" | would need converting to `LifecycleNode` to gain configure/activate/deactivate — bigger lift, not needed for heartbeat/register alone |
| `config/<subsystem>_parameters.yaml` per subsystem | `config/thermal_nodes.yaml`, `config/thermals.yaml` already split similarly | already close to this shape |
| `test/unit/`, `test/integration/`, `test/ros/` | no tests currently | would need to be added from scratch |

This is a bigger, multi-step migration than the current `REFACTOR_PLAN.md`
scope (which only adds the heartbeat/register/fault trio to one node). Worth
treating as a separate, later refactor once the registration piece lands and
proves the pattern out.
