# `ssos_thermal`: Thermal Control on the SSOS subsystem backbone

## TL;DR

`ssos_thermal` is a **new package**, added in parallel with the legacy
`space_station_thermal_control`, which this work does not edit or remove.
It follows the `ssos_eclss` pattern (see
[ECLSS_PATTERN_REFERENCE.md](ECLSS_PATTERN_REFERENCE.md)):

- ROS-independent physics libraries for the thermal network and the
  coolant loop
- thin ROS 2 `LifecycleNode` wrappers: `thermal_network` and `coolant_node`
- registration with `system_manager`, heartbeats, telemetry, and an
  edge-triggered over-temperature fault
- coolant command over the `/coolant_heat_transfer` action (only
  `thermal_network` sends goals) and coolant monitoring over
  `/thermal/coolant/status`
- a reduced, representative 3-node thermal graph
- a standalone launch file plus integration into the full-station launch
- mission-control GUI compatibility (Thermal panel and subsystem roster)
- unit tests for both physics models and ROS lifecycle tests on isolated
  `ROS_DOMAIN_ID`s

Target: v0.9.x, merging into `v0.9.1-dev`.

## Package layout

```
ssos_thermal/
├── include/ssos_thermal/
│   ├── network/thermal_network.hpp     Layer 2 — thermal network physics, NO ROS
│   ├── coolant/coolant_loop.hpp        Layer 2 — coolant loop physics, NO ROS
│   └── nodes/
│       ├── thermal_network_node.hpp    Layer 3 — LifecycleNode wrapper
│       ├── coolant_node.hpp            Layer 3 — LifecycleNode wrapper
│       └── thermal_diagnostics.hpp     Layer 3 — heartbeat/fault/autostart helper (shared)
├── src/
│   ├── network/thermal_network.cpp     \  thermal_network_physics library
│   ├── coolant/coolant_loop.cpp        /
│   ├── nodes/*.cpp                        thermal_network_ros library
│   └── main/{thermal_network,coolant}_main.cpp   executables
├── config/
│   ├── thermal_network.yaml            thermal_network parameters
│   ├── thermal_nodes.yaml              node/link graph (3 nodes)
│   └── coolant.yaml                    coolant_node parameters
├── launch/thermal.launch.py
├── scripts/thermal_visualization.py    standalone PyQt5 network viewer
├── test/
│   ├── unit/network/test_thermal_network.cpp
│   ├── unit/coolant/test_coolant_loop.cpp
│   ├── unit/diagnostics/test_thermal_diagnostics.cpp
│   ├── ros/test_thermal_network_node.cpp
│   └── ros/test_coolant_node.cpp
└── docs/{architecture,parameters,commands,fault_catalog}.md
```

```
 thermal_network_physics   (yaml-cpp only — links nothing from ROS)
   ▲
   │ link + wrap
 thermal_network_ros       (rclcpp, rclcpp_lifecycle, rclcpp_action,
   ▲                        ament_index_cpp, space_station_interfaces)
   │ link
 thermal_network_node, coolant_node   (executables)
```

This mirrors `eclss_physics` / `eclss_ros`. Only the ROS layer resolves the
config file's path via `ament_index_cpp`; the physics library receives a
plain path string, so the same models could run without ROS.

## 1. Physics libraries (no ROS)

**`ssos_thermal::network::ThermalNetwork`**

- `load_from_yaml(filepath, reference_temp_c = 20.0)` loads nodes
  (`node_name`, `heat_capacity`, `internal_power`) and one conductive link
  per node to its `parent_link`. An empty `parent_link` marks the root (no
  link). A link only conducts if both ends are declared nodes.
- `step(dt)` integrates all nodes together with classic RK4:
  `C_i · dT_i/dt = P_i + Σ_j k_ij · (T_j − T_i)`.
- `link_heat_flow(link)` returns `k · (T_from − T_to)`, the same conduction
  term, for `/thermal/links/flux` telemetry.
- `average_temperature()` drives the cooling trigger; `hottest()` drives
  diagnostics and the over-temperature fault; `set_all_temperatures()`
  applies coolant feedback.

**`ssos_thermal::coolant::CoolantLoop`**

- `CoolantParams { mass_kg, specific_heat_j_per_kg_c,
  heat_transfer_efficiency, vent_threshold_kj }`.
- `step(node_temp_c, target_temp_c)` performs one cooldown step: moves the
  temperature at most 2.5 °C toward the target, transfers the removed heat
  to the ammonia loop at `heat_transfer_efficiency`, and flags venting at
  `vent_threshold_kj`. Ported from the only code path of the legacy
  `CoolantActionServer` that anything used (see change history item 2).
- `kAmmoniaBaseTempC` is the ammonia temperature with no heat transferred,
  used for the idle status.

The formulas are written out in [docs/architecture.md](docs/architecture.md).

## 2. ROS wrappers (`LifecycleNode`)

Both nodes self-configure and self-activate via
`ThermalDiagnostics::maybe_autostart` (gated on the `autostart` parameter,
after `autostart_delay_ms`), so no launch file emits lifecycle
`ChangeState` events — the same mechanism `ssos_eclss` uses.

**`thermal_network`** (`ThermalNetworkNode`)

| Callback | Does |
|---|---|
| `on_configure` | Read parameters; load the network from `thermal_config_file`; create lifecycle publishers for node state, link flux, diagnostics, heartbeat, and faults; create the `Coolant` action client and the registration client |
| `on_activate` | Activate publishers, start the step timer at `thermal_update_dt`, register as `"thermal"` |
| `on_deactivate` | Cancel the timer, deactivate publishers |
| `on_cleanup` | Release everything |
| each tick | Send one cooling goal when `average_temperature() > cooling_trigger_threshold`; step the network unless a cooling goal is in flight; publish node state, link flux, and diagnostics; publish a heartbeat; publish a fault only on the healthy→unhealthy transition |

**`coolant_node`** (`CoolantNode`)

| Callback | Does |
|---|---|
| `on_configure` | Build `CoolantLoop` from parameters; create the `/coolant_heat_transfer` action server, the status/heartbeat/fault publishers, the best-effort radiator `VentHeat` client, and the registration client; set idle status |
| `on_activate` | Activate publishers, start a 1 Hz heartbeat + status timer, register as `"coolant"` |
| `on_deactivate` | Stop and join the goal thread (a running goal is aborted), cancel the timer, deactivate publishers |
| `on_cleanup` | Same, then release everything |
| goal | Rejected unless the node is `ACTIVE` and no goal is running. Runs on a node-owned thread: steps `CoolantLoop` until within 0.5 °C of `target_temp_c`, publishing action feedback and `/thermal/coolant/status` each step, then calls the radiator's `VentHeat` best-effort if heat was vented |

**`ThermalDiagnostics`** (shared): `make_heartbeat(stamp, subsystem_name,
lifecycle_state, healthy, status_message)`, `make_fault(stamp,
subsystem_name, fault_type, severity, description, affected_interfaces)`,
`should_raise_fault(healthy)` (true once per healthy→unhealthy transition),
and `maybe_autostart(node, delay_ms)`.

## 3. Interfaces

| Name | Type | Direction |
|---|---|---|
| `/ssos/register_subsystem` | `RegisterSubsystem` srv | both nodes → `system_manager` |
| `/ssos/thermal/heartbeat`, `/ssos/coolant/heartbeat` | `SubsystemHeartbeat` | → `system_manager`, GUI roster |
| `/ssos/fault_event` | `FaultEvent` | `thermal_network` → `system_manager` |
| `/thermal/nodes/state` | `ThermalNodeDataArray` | `thermal_network` → GUI |
| `/thermal/links/flux` | `ThermalLinkFlowsArray` | `thermal_network` → GUI |
| `/thermals/diagnostics` | `DiagnosticStatus` | `thermal_network` → telemetry |
| `/coolant_heat_transfer` | `Coolant` action | `thermal_network` → `coolant_node` (command) |
| `/thermal/coolant/status` | `CoolantStatus` (new) | `coolant_node` → GUI (monitoring) |
| `/tcs/radiator_a/vent_heat` | `VentHeat` srv | `coolant_node` → legacy `radiator` (best-effort) |

`CoolantStatus.msg` is the only new interface, added to
`space_station_interfaces/atcs/msg/` as all interfaces are.

## 4. Launch and integration

- `launch/thermal.launch.py` starts `thermal_network` and `coolant_node`
  (`autostart: true`, `autostart_delay_ms` launch argument, default 300) and,
  unless `launch_visualization:=false`, the standalone
  `thermal_visualization.py` viewer.
- `space_station/launch/space_station.launch.py` includes it with
  `autostart_delay_ms: 11000` (after `system_manager` and
  `simulation_controller` activate at t = 10 s) and
  `launch_visualization: false`.
- `pixi.toml`: `ssos_thermal` is part of both the `build` and `test` tasks.
- Mission-control GUI: `space_station/space_station/thermal.py` subscribes to
  node state, link flux, and coolant status (it sends no goals);
  `main_window.py` rolls the `thermal` and `coolant` heartbeats into one
  "THERMAL" roster row (unhealthy if either is).

## 5. Tests

| Binary | Covers |
|---|---|
| `test_thermal_network` | YAML loading, root-node handling, conduction direction, `hottest()`, link heat flow sign and inert links |
| `test_coolant_loop` | Step size cap, convergence to target, vent threshold |
| `test_thermal_diagnostics` | Fault edge trigger: once per transition, silent while active, again after recovery |
| `test_thermal_network_node` | configure/activate/deactivate/cleanup, autostart |
| `test_coolant_node` | configure/activate/deactivate/cleanup, autostart, idle status without a goal, active→idle status around a goal, deactivate mid-goal aborts within 1 s |

The two ROS-based binaries run on their own `ROS_DOMAIN_ID`s (214, 215) so
their real `/ssos/register_subsystem` calls can't reach another package's
test when CI runs packages in parallel.

## 6. Documentation

[README.md](README.md), [docs/architecture.md](docs/architecture.md)
(layers, model formulas, data flow, lifecycle),
[docs/parameters.md](docs/parameters.md),
[docs/commands.md](docs/commands.md), and
[docs/fault_catalog.md](docs/fault_catalog.md).

## Change history

The sections below record each scope change in the order it was made.

### 1. Port `sun_vector` / `array_absorptivity` Bullet-free (later removed, see 4)

**`space_station_thermal_control` is not edited by this plan at all** —
same rule as `ssos_eclss` never touching `space_station_eclss`. Its
`sun_vector.hpp`/`.cpp` and `solar_heat_node.hpp`/`.cpp` (the
`array_absorptivity` executable) keep running exactly as they are today,
Bullet dependency and all.

Instead, `ssos_thermal` gets **new** files — fresh ports of the same logic,
Bullet-free from the start — alongside the `thermal_network` migration
above:

- `include/ssos_thermal/nodes/sun_vector_node.hpp` / `src/nodes/sun_vector_node.cpp`
- `include/ssos_thermal/nodes/solar_heat_node.hpp` / `src/nodes/solar_heat_node.cpp`

These pull in `<bullet/LinearMath/btVector3.h>` / `btQuaternion.h` in the
legacy package purely for vector dot products, normalization, and one
quaternion-conjugation rotation (`q_body_inv * s_quat * q_body` in
`sun_vector.cpp`, to express the sun vector in body frame). None of
Bullet's actual physics/collision machinery is used — this is a
geometry-math dependency, not a physics-engine one, so the ports don't
carry it forward.

**Decision:** replace `btVector3`/`btQuaternion` with small, std-only
`Vector3`/`Quaternion` structs (option evaluated against matching
`ssos_eclss`'s `std::vector<double>` state-array convention — rejected
because that convention fits *variable-length* discretized physics state
(bed depth nodes), not fixed 3-component geometry; a bare
`std::vector<double>` here would mean indexing by `v[0]`/`v[1]`/`v[2]` with
no compile-time guard against mixing up a 3-vector and a 4-quaternion).

```cpp
// ssos_thermal::math3d — new header, no ROS/Bullet deps
struct Vector3 {
  double x, y, z;
  Vector3 operator-(const Vector3 &o) const;
  double dot(const Vector3 &o) const;
  double length() const;
  Vector3 normalized() const;
  void normalize();
};

struct Quaternion {
  double x, y, z, w;
  Quaternion inverse() const;              // conjugate; valid for unit quaternions
  Quaternion operator*(const Quaternion &o) const;  // Hamilton product
};
```

Affected files (all new, under `ssos_thermal/`):

- **New:** `include/ssos_thermal/math3d.hpp` — the `Vector3`/`Quaternion`
  structs above.
- **New:** `include/ssos_thermal/nodes/sun_vector_node.hpp` +
  `src/nodes/sun_vector_node.cpp` — same logic as the legacy
  `SunVectorProvider`/`tryComputeSunVector()`, with `btVector3`/`btQuaternion`
  replaced by `math3d::Vector3`/`math3d::Quaternion` throughout.
- **New:** `include/ssos_thermal/nodes/solar_heat_node.hpp` +
  `src/nodes/solar_heat_node.cpp` — same logic as the legacy
  `SolarHeatNode`, `PanelParams::normal` and `computePanelHeat`'s `sun_dir`
  parameter typed as `math3d::Vector3` instead of `btVector3`.
- **New:** `src/main/sun_vector_main.cpp`, `src/main/solar_heat_main.cpp` —
  thin mains, mirroring `thermal_network_main.cpp`.
- `ssos_thermal/CMakeLists.txt` gets `sun_vector` and `array_absorptivity`
  (or their `ssos_thermal`-side executable names) as plain `add_executable`
  targets with **no Bullet find_package/link** at all — there was never a
  `find_package(Bullet)` in the legacy `CMakeLists.txt` either (its
  `${BULLET_LIBRARIES}` reference resolves empty/undefined today), so this
  isn't replacing a working Bullet integration, just not carrying the
  reference forward.

Both ported nodes stay plain `rclcpp::Node`s — this port is scoped to the
math dependency only, not a lifecycle/registration conversion (that stays
scoped to `thermal_network`, as above). `space_station_thermal_control`'s
`sun_vector` and `array_absorptivity` executables are untouched and keep
running as-is; nothing removes them.

**Update — later removed:** the orbit-calculation piece (`sun_vector_node`,
Julian-date/ECI sun-position math) was pulled back out of `ssos_thermal`
entirely; see change history item 4 below.

### 2. Port `cooling_server` as `coolant_node`

**Why:** `ThermalNetworkNode`'s cooling client depends on the
`/coolant_heat_transfer` action, and the mission-control GUI's coolant
cards (Internal Temp / Ammonia Temp / Vented Heat) need coolant-loop data.
The only action server was `space_station_thermal_control`'s
`cooling_server`, which wasn't launched anywhere in this migration.
`space_station_thermal_control` isn't edited (same rule as every other
section here) — `ssos_thermal` gets a **new**, from-scratch port instead:
`coolant_node`.

**What was actually ported, and what wasn't.** Reading `cooling.cpp`
end to end, the *only* code path anything downstream depends on is
`CoolantActionServer::execute()` — the goal-driven cooldown loop that
produces the `internal_temp_c`/`ammonia_temp_c`/`vented_heat_kj` feedback.
Everything else in that class is inert:

| Legacy member | Why it's dead code |
|---|---|
| `tickBehaviorTree()` + `isTempHigh()`/`isAmmoniaHot()` | Their condition variables (`current_temp_`, `ammonia_heat_kj_`) are never written anywhere else in the class — the BT always evaluates the same static branch |
| `ventHeat()`/`refreshWater()` BT actions | Their clients (`vent_client_`, `wrs_client_`) are declared but never constructed — always `nullptr`, so these leaves always fail |
| `recycleWater()` | Never called from anywhere |
| `publishInternalLoop()` / `publishExternalLoop()` | Declared publishers, never actually published to |

None of this was carried forward. `coolant_node` ports only the real
behavior: the cooldown physics (extracted to `CoolantLoop`, zero ROS) and
the action server wrapping it. `radiator_client_` *is* kept (it's the one
thing `execute()` actually calls, best-effort, when venting occurs).

**Files** (all new, under `ssos_thermal/`):

- `include/ssos_thermal/coolant/coolant_loop.hpp` + `src/coolant/coolant_loop.cpp`
  — `CoolantLoop::step(node_temp_c, target_temp_c)`, one physics step
  (matches the legacy per-iteration body: ≤2.5 degC toward target, heat
  transferred to ammonia at `heat_transfer_efficiency`, vent flag at
  `vent_threshold_kj`).
- `include/ssos_thermal/nodes/coolant_node.hpp` + `src/nodes/coolant_node.cpp`
  — `CoolantNode : rclcpp_lifecycle::LifecycleNode`, same shape as
  `ThermalNetworkNode`: `maybe_autostart`, register-as-`"coolant"`,
  `/ssos/coolant/heartbeat` on a 1 Hz timer (goal execution is separate
  from the heartbeat cadence, since cooling is goal-driven not periodic).
- `src/main/coolant_main.cpp`.
- `config/coolant.yaml`.
- `test/unit/coolant/test_coolant_loop.cpp` (physics-only), `test/ros/test_coolant_node.cpp`
  (lifecycle + autostart, idle status without any goal, active→idle status
  around a real goal, deactivate mid-cycle aborts promptly).

**`ThermalDiagnostics` grew up:** now that two nodes register under
different subsystem names, `make_heartbeat`/`make_fault` take
`subsystem_name` as a parameter instead of hardcoding `"thermal"` —
`thermal_network_node.cpp`'s call sites were updated to pass `"thermal"`
explicitly. Same evolution `EclssDiagnostics` went through going from one
caller to five.

**GUI monitoring is separate from coolant command (`/thermal/coolant/status`).**
The legacy `ThermalWidget` got its card data by *sending* a one-shot
`Coolant` goal (`input_temperature_c = 30.0`) and reading the action
feedback. That design had three problems, all found in review:

- Opening the GUI actuated the coolant loop and, whenever venting
  triggered, called the radiator's `VentHeat` — monitoring caused command.
- The rclpy feedback callback receives the `FeedbackMessage` wrapper
  (`goal_id` + `.feedback`), not the payload; the widget read fields off
  the wrapper, so the callback failed and the cards stayed "NO DATA".
- An interim "fix" here (polling `server_is_ready()` so the goal survived
  `coolant_node`'s 11 s autostart delay) only made the actuation reliable;
  it was wrongly recorded as verified on the strength of the server-side
  `[ACTION] Cooling goal received` log, not the cards themselves.

Replaced with a read-only status topic: `coolant_node` publishes
`space_station_interfaces/msg/CoolantStatus` (`active`, `component_id`,
`internal_temp_c`, `ammonia_temp_c`, `vented_heat_kj`) on
`/thermal/coolant/status` at 1 Hz and on every cooldown step. Before the
first goal it reports idle values (loop at `target_temp_c`, ammonia at
`CoolantLoop::kAmmoniaBaseTempC`, nothing vented). `ThermalWidget` only
subscribes — it has no action client — and its cards show "IDLE" or
"COOLING". Only `thermal_network` sends `Coolant` goals.

**Goal-execution thread is owned, not detached.** The legacy server ran
`execute()` on a detached thread. After a cycle with venting, that thread
waits on the radiator service; if the node was cleaned up or destroyed in
the meantime, the thread used a freed object (found as a segfault at test
exit). `execute_thread_` is now a member, joined on deactivate, cleanup and
destruction; `stop_requested_` lets a running cycle exit at the next step
and abort its goal, and the radiator waits poll that flag instead of
blocking for their full timeout. A new goal is rejected while one runs.

**Launch:** `coolant_node` added to `launch/thermal.launch.py` as a second
`LifecycleNode` alongside `thermal_network`, same `autostart`/
`autostart_delay_ms` treatment. `main_window.py`'s `_SUBSYS_ALIASES` maps
both `"thermal"` and `"coolant"` to the roster's single "Thermal" row; its
`_apply_heartbeat` aggregation was generalized (mirroring the existing
ECLSS multi-node aggregation) so the two heartbeats don't just overwrite
each other's roster status.

When a cycle vents and `radiator` isn't running (as in the full-station
launch), the venting step logs `[RADIATOR] VentHeat service not available`
— graceful degradation, not a failure.

### 3. Reduce the node graph to 3 components, fix a dead-link bug

`config/thermal_nodes.yaml` originally carried ~46 lumped equipment nodes
plus `SolarPanel1`/`SolarPanel2`, mirroring the legacy solver's synthetic
demo config. Reduced to 3 nodes: `base_link` (the station structure, one
lumped mass standing in for every interior subsystem/avionics load the
other 46 entries used to represent individually) plus the two solar panels.

**Bug found and fixed in the process:** every one of the original 46
entries pointed `parent_link` at `"base_link"` or at another entry, but
`"base_link"` itself was never declared as a `node_name` anywhere in the
file. `ThermalNetwork::compute_dTdt()` only exchanges heat between names
present in its node map, so a link whose `to` isn't a declared node is
inert — every top-level component's conduction to "the structure" was
silently a no-op; only parent/child pairs *within* the equipment tree
(e.g. `Camera1` → `MainComputer`) ever actually exchanged heat. This was
invisible before because the network had enough internal complexity that
nothing exercised the top-level case directly.

Fixed two ways:
- `base_link` is now a real declared node (`heat_capacity`/`internal_power`
  are the **sums** of the previous 46 non-panel entries — 30300.0 J/°C and
  1380.0 W — so the aggregate thermal mass and power budget this network
  represents is unchanged, only the topology is simplified to a 3-node
  star: `SolarPanel1`/`SolarPanel2` each conduct to `base_link`).
- `ThermalNetwork::load_from_yaml()` now treats an empty (or omitted)
  `parent_link` as "this is the root, no link" instead of creating a link
  to an implicit, undeclared name — so a future root node can't silently
  reproduce the same bug. `base_link`'s own `parent_link: ""` uses this.

**Explicitly not done** in this pass (approved scope was the node-count
reduction + the base_link fix only): `SolarPanel1`/`SolarPanel2` still use
a constant YAML `internal_power` (10.0 W each, a stand-in for wiring
resistive losses) rather than the real per-panel absorbed solar power
`SolarHeatNode` already computes on `/thermal/solar_heat` — `ThermalNetworkNode`
does not subscribe to that topic. No radiative heat-rejection term exists
either, so nothing in this network can shed heat to space; the only heat
sink for the whole graph remains the coolant-loop action's
`set_all_temperatures()` snap-down, same as before this change. Both are
natural follow-ups if solar-driven panel temperatures are wanted later.

### 4. Remove the orbit and solar-heat nodes

`sun_vector_node` (`include/ssos_thermal/nodes/sun_vector_node.hpp` +
`src/nodes/sun_vector_node.cpp`) computed Julian date, Julian centuries
since J2000.0, and a low-precision solar ephemeris (unit sun vector in the
ECI frame) from ROS time, then rotated it into the spacecraft body frame
using `/gnc/pose_all`. That's orbital-mechanics/spacecraft-attitude math,
not thermal physics — it belongs with GNC (which already owns
`orbit_dynamics` and spacecraft pose), not `ssos_thermal`. Removed
entirely: `sun_vector_node.hpp`/`.cpp`, its `add_executable` and install
entry in `CMakeLists.txt`, and its `Node` entry in `launch/thermal.launch.py`.

`math3d::Quaternion` was removed from `include/ssos_thermal/math3d.hpp`
alongside it — it existed solely for `sun_vector_node`'s body-frame
rotation (`q_body_inv * s_quat * q_body`); nothing else in the package uses
quaternions. `math3d::Vector3` stays — `solar_heat_node` still uses it for
the panel-normal/sun-direction dot product.

**Update — `solar_heat_node` also later removed:** with nothing publishing
`/sun_vector_body`, `solar_heat_node` had no way to ever produce output —
`sunVectorCallback()` could never fire, so it was permanently idle, not
just temporarily. Reviewing this doc's own architecture made that dead
end obvious, so `solar_heat_node.hpp`/`.cpp` were removed too (along with
their `add_executable`/install entries, its `Node` entry in
`launch/thermal.launch.py`, and `find_package(geometry_msgs)`, which
nothing else in the package needed). `math3d::Vector3` — its only
remaining user — was removed alongside it, so `math3d.hpp` is gone
entirely. `ssos_thermal` now has exactly two nodes: `thermal_network` and
`coolant_node`. If solar heating is wanted later, both the sun-vector
source and the panel-absorption calculation would need to be reintroduced
together, ideally with `thermal_network` actually wired to consume the
result this time (see "Out of scope" below).

## Out of scope

Matches the issue's out-of-scope list:

- [ ] Full migration or removal of `space_station_thermal_control` — it
      stays available and unmodified.
- [ ] High-fidelity spacecraft thermal modeling — the network is a reduced,
      representative 3-node conductive model.
- [ ] Radiation-to-space modeling — no node has a radiative term; the only
      heat sink is the coolant loop.
- [ ] Detailed solar-heating or orbital-environment modeling — the solar
      panels heat only from their YAML `internal_power` plus conduction to
      `base_link`; the sun-vector and panel-absorption ports were removed
      (see the change history above).
- [ ] Direct `/sim/world_state` thermal-environment coupling —
      `thermal_network` subscribes to no simulation input.
- [ ] High-fidelity coolant or ammonia-loop validation — the ammonia
      temperature relation is the legacy simplified model, not validated.
- [ ] Migration of the legacy `radiator`, `demand`, `sun_vector`, or
      `array_absorptivity` nodes — they stay in
      `space_station_thermal_control`; `coolant_node` only calls the
      `radiator` service best-effort.
- [ ] Full thermal-control Behavior Tree migration — the legacy
      `cooling_server` BT was inert and was not ported.
- [ ] Complete replacement of legacy thermal functionality.

Also not done in this package (follow-ups):

- [ ] Fault-injection scenario integration — no YAML-schedulable faults for
      thermal/coolant, unlike `ssos_eclss`'s `FaultInjector`.
- [ ] A fault model for `coolant_node` (always reports `healthy=true`) — see
      `docs/fault_catalog.md`.

## Verification checklist

- [x] `space_station_thermal_control` is unchanged: `git diff
      origin/v0.9.1-dev...HEAD -- space_station_thermal_control` is empty
- [x] `thermal_network_physics` references no ROS symbols (`nm
      --undefined-only` shows no `rclcpp`/`rcl`/`rmw`/interface symbols)
- [x] `pixi run build` compiles `ssos_thermal` clean (added to the pixi
      task's `--packages-up-to` list)
- [x] CI test set `colcon test --packages-select-regex '^ssos_.*$'` and
      `pixi run test` both pass: 186 tests, 0 errors, 0 failures
- [x] `colcon test --packages-select ssos_thermal` passes — 5 test binaries
      (`test_thermal_network`, `test_coolant_loop`, `test_thermal_diagnostics`,
      `test_thermal_network_node`, `test_coolant_node`); also part of
      `pixi run test`
- [x] ROS-based test binaries run on their own `ROS_DOMAIN_ID`s, so their
      `/ssos/register_subsystem` calls can't reach another package's test
      (found when CI ran `ssos_core` and `ssos_thermal` tests in parallel)
- [x] `ros2 launch ssos_thermal thermal.launch.py` (standalone) and the
      full `space_station.launch.py` both bring `/thermal_network` and
      `/coolant_node` to `active` with **no manual lifecycle call**
      (confirms `maybe_autostart` works for both)
- [x] `ros2 topic echo /ssos/thermal/heartbeat` and `/ssos/coolant/heartbeat`
      show periodic `healthy=true`, `lifecycle_state=2 (ACTIVE)`
- [x] With `ssos_core`'s `system_manager` running via the full-station
      launch: `Registered subsystem 'thermal'` and `'coolant'` both log, no
      `"system_manager registration service unavailable"` warning, and
      `SystemState` reaches `NOMINAL`
- [x] `ros2 param set /thermal_network enable_failure true` + lowered
      `max_temp_threshold` → exactly **one** `FaultEvent` on
      `/ssos/fault_event` at the transition (not one per tick); the edge
      trigger is also unit-tested (`test_thermal_diagnostics`)
- [x] `/thermal/nodes/state` and `/thermal/links/flux` publish the same
      data shape the legacy solver did — the GUI's `ThermalWidget` node/link
      tables and plot work unmodified
- [x] `/thermal/links/flux` `heat_flow` is `k·(T_from − T_to)` from both
      connected nodes (`ThermalNetwork::link_heat_flow`, the same relation
      `step()` integrates), not a comparison against a fixed 20 °C
- [x] GUI roster shows "THERMAL — NOMINAL" (screenshot-verified), driven
      live by `/ssos/thermal/heartbeat` + `/ssos/coolant/heartbeat`
      aggregation, not a placeholder
- [x] GUI's Internal Temp / Ammonia Temp / Vented Heat cards show live
      `/thermal/coolant/status` data ("IDLE" before any cooling cycle), and
      opening the GUI sends no `/coolant_heat_transfer` goal — checked on
      the full-station launch: no `Cooling goal received` in the log, and a
      `ThermalWidget` rendered against the live topics shows 25.0 °C /
      5.0 °C / 0.0 kJ, all "IDLE"
- [x] Docs cover architecture and model formulas, parameters, commands, and
      fault behavior (`docs/`, `README.md`)
