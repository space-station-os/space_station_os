# Refactor: bring `thermal_nodes` up to the `ssos_eclss` pattern

## TL;DR

Two earlier passes at this plan only bolted a heartbeat/register/fault
trio onto the existing `ThermalSolverNode`. This revision goes further, per
[ECLSS_PATTERN_REFERENCE.md](ECLSS_PATTERN_REFERENCE.md): **adopt the actual
ssos_eclss architecture** for the one node that matters to the GUI/roster —
physics extracted into a ROS-free library, the node converted to a
`LifecycleNode`, and a shared `ThermalDiagnostics` helper — not just the
registration side-effect.

**Scope stays at one node**: `thermal_nodes` (currently `ThermalSolverNode`)
is the only executable touched. `cooling_server`, `radiator`, `demand`, and
`array_absorptivity` are untouched — nothing downstream (GUI, roster,
`system_manager`) depends on them individually today, and converting five
executables at once is a much bigger effort than this pass justifies. If
that changes later, `ThermalDiagnostics` is already written to be reusable.

**Still not in this pass:** GUI wiring (`main_window.py`), and
`space_station.launch.py` / `thermals.launch.py` changes — the executable
and ROS node names stay the same (`thermal_nodes` / `thermal_network`), so
existing launch files keep working unmodified.

## Target architecture

```
space_station_thermal_control/
├── include/space_station_thermal_control/
│   ├── network/thermal_network.hpp     Layer 2 — physics, NO ROS      \
│   ├── nodes/thermal_network_node.hpp  Layer 3 — LifecycleNode wrapper |
│   ├── nodes/thermal_diagnostics.hpp   Layer 3 — heartbeat/fault/     |
│   │                                    autostart helper (shared)     |
│   └── {cooling,radiators,...}.hpp     untouched legacy nodes         |
├── src/
│   ├── network/thermal_network.cpp     thermal_network_physics lib
│   ├── nodes/thermal_network_node.cpp  \
│   ├── nodes/thermal_diagnostics.cpp    thermal_network_ros lib
│   ├── main/thermal_network_main.cpp   thermal_nodes executable
│   └── {cooling,radiators,...}.cpp     untouched legacy executables
├── test/                                [new] mirrors ssos_eclss/test/
│   ├── unit/network/test_thermal_network.cpp
│   └── ros/test_thermal_network_node.cpp
└── config/thermal_nodes.yaml            unchanged — already close to
                                          ssos_eclss's one-YAML-per-subsystem
```

```
 thermal_network_physics  <───────────┐
 (zero ROS — yaml-cpp only)           │ link only this
   ▲                                  │
   │ link + wrap                       (future: standalone validation
   │                                    tool, not needed yet)
 thermal_network_ros
 (rclcpp, rclcpp_lifecycle, rclcpp_action, space_station_interfaces)
   ▲
   │ link
 thermal_nodes  (executable — same name as today, launch files unchanged)
```

This mirrors `eclss_physics` / `eclss_ros` exactly, minus the multi-node
sharing concern (ssos_eclss has 5 nodes sharing `eclss_ros`; thermal has 1
in scope, so there's no separate library needed on the ROS side beyond what
one executable needs — `thermal_network_ros` exists mainly to keep the
mirror obvious and leave room to grow).

## What moves where

### 1. New physics library: `thermal_network` (no ROS)

Extract from today's `ThermalSolverNode` into
`include/.../network/thermal_network.hpp` + `src/network/thermal_network.cpp`,
namespace `space_station_thermal_control::network`:

- `struct ThermalNodeState { temperature, heat_capacity, internal_power; }`
  and `struct ThermalLinkState { from, to, joint_name, conductance; }` —
  same fields as today's `ThermalNode`/`ThermalLink` structs, just relocated.
- `class ThermalNetwork` owning `nodes_`/`links_`, with:
  - `static ThermalNetwork load_from_yaml(const std::string &filepath);` —
    the existing `parseYAMLConfig` logic, moved here. This is legitimately
    ROS-free: `yaml-cpp` has zero ROS dependency (only resolving the
    package-share file *path* via `ament_index_cpp` stays in the node
    layer, same split ssos_eclss uses for everything else).
  - `void step(double dt);` — the existing RK4 integration
    (`compute_dTdt` + the `k1..k4` loop from `updateSimulation()`), moved
    here verbatim.
  - `const std::unordered_map<std::string, ThermalNodeState> &nodes() const;`
    and `const std::vector<ThermalLinkState> &links() const;` accessors.
  - `struct Hottest { std::string name; double temperature; };` +
    `Hottest hottest() const;` — replaces the duplicated hottest-node scan
    that today happens twice (once in `updateSimulation()`, again in
    `publishThermalNetworkDiag`).

Cooling-feedback and diagnostics/heartbeat message-building stay **out** of
this library — they're ROS/action concerns, not physics.

### 2. `ThermalSolverNode` → `ThermalNetworkNode`, a `LifecycleNode`

`include/.../nodes/thermal_network_node.hpp` +
`src/nodes/thermal_network_node.cpp`, namespace
`space_station_thermal_control::nodes`. Same shape as `ArsNode`:

| Lifecycle callback | Does |
|---|---|
| Constructor | Declare parameters (`enable_failure`, `enable_cooling`, thresholds, `thermal_update_dt`, `thermal_config_file` — unchanged) + call `ThermalDiagnostics::maybe_autostart(this)` |
| `on_configure()` | `network_ = ThermalNetwork::load_from_yaml(...)`; create all publishers as `LifecyclePublisher` (`node_pub_`, `link_pub_`, `diag_pub_`, `heartbeat_pub_`, `fault_pub_`); create `cooling_client_` (rclcpp_action, unaffected by lifecycle) and `register_client_`; add param-validation callback |
| `on_activate()` | Activate all `LifecyclePublisher`s, start `timer_` → `updateSimulation()`, call `registerWithManager()` |
| `on_deactivate()` | Deactivate publishers, cancel `timer_` |
| `on_cleanup()` | Reset everything |
| `updateSimulation()` | `network_->step(dt)`, run `coolingCallback()` against `network_->nodes()`, publish node/link/diag messages, then `healthy = !(enable_failure_ && network_->hottest().temperature > max_temp_threshold_)` → `heartbeat_pub_`/`fault_pub_` via `ThermalDiagnostics` |
| `registerWithManager()` | Identical shape to `ArsNode::register_with_manager()` — wait ≤200ms for `/ssos/register_subsystem`, warn-and-continue if absent, send `{subsystem_name="thermal", published_topics, subscribed_topics={}, heartbeat_topic="/ssos/thermal/heartbeat"}` |

**Why `maybe_autostart` instead of relying on an external orchestrator:**
neither `thermals.launch.py` nor `space_station.launch.py` drives lifecycle
`ChangeState` events for thermal nodes today (unlike
`space_station.launch.py`'s `activate_core_sim` TimerAction for
`system_manager`/`simulation_controller`). Using the same self-activating
one-shot timer `ArsNode` uses means the `LifecycleNode` conversion needs
**zero launch-file changes** — `ros2 launch space_station_thermal_control
thermals.launch.py` keeps working exactly as before, just with the node
spending its first ~300ms unconfigured before self-activating.

### 3. New shared helper: `ThermalDiagnostics`

`include/.../nodes/thermal_diagnostics.hpp` +
`src/nodes/thermal_diagnostics.cpp` — same role as `EclssDiagnostics`, sized
for one subsystem (so `subsystem_name` is baked in as `"thermal"` rather
than a parameter, since there's only one caller today — unlike
`EclssDiagnostics` which is shared across 5 differently-named subsystems):

```cpp
class ThermalDiagnostics
{
public:
  static space_station_interfaces::msg::SubsystemHeartbeat make_heartbeat(
    const rclcpp::Time &stamp, bool healthy, const std::string &status_message);

  static space_station_interfaces::msg::FaultEvent make_fault(
    const rclcpp::Time &stamp, const std::string &fault_type, uint8_t severity,
    const std::string &description,
    const std::vector<std::string> &affected_interfaces = {});

  // Edge-detection: call every tick, returns true exactly once per
  // healthy->unhealthy transition (mirrors ArsNode's was_healthy_ member,
  // but encapsulated so ThermalNetworkNode doesn't hand-roll it).
  bool should_raise_fault(bool healthy);

private:
  bool was_healthy_ = true;
};

  // One-shot timer: self-configure + self-activate shortly after
  // construction, so a launch file doesn't need lifecycle ChangeState
  // events. Same contract as ssos_eclss's EclssDiagnostics::maybe_autostart.
  rclcpp::TimerBase::SharedPtr maybe_autostart(
    rclcpp_lifecycle::LifecycleNode *node, int delay_ms = 300);
```

### 4. Build files

`CMakeLists.txt`:

```cmake
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(thermal_network_physics src/network/thermal_network.cpp)
target_link_libraries(thermal_network_physics yaml-cpp)
target_include_directories(thermal_network_physics PUBLIC include)
# NOTE: thermal_network_physics links NOTHING from ROS

add_library(thermal_network_ros
  src/nodes/thermal_network_node.cpp
  src/nodes/thermal_diagnostics.cpp
)
target_link_libraries(thermal_network_ros thermal_network_physics ${cpp_typesupport_target})
ament_target_dependencies(thermal_network_ros
  rclcpp rclcpp_lifecycle rclcpp_action ament_index_cpp
  space_station_interfaces diagnostic_msgs lifecycle_msgs)

add_executable(thermal_nodes src/main/thermal_network_main.cpp)
target_link_libraries(thermal_nodes thermal_network_ros)
```

This **replaces** the current `add_executable(thermal_nodes
src/thermals_solver.cpp)` block. Executable name (`thermal_nodes`) and the
`install(TARGETS ...)` destination are unchanged, so nothing outside this
package needs to know the internals moved.

`package.xml`: add `<depend>rclcpp_lifecycle</depend>` and
`<depend>lifecycle_msgs</depend>` (new — the package doesn't use lifecycle
today).

### 5. Tests (new — package currently has none)

Mirrors `ssos_eclss/test/unit/` and `test/ros/`:

- `test/unit/network/test_thermal_network.cpp` — gtest, links
  `thermal_network_physics` only. Covers: YAML loading produces the
  expected node/link count, one `step()` moves temperature in the correct
  direction for a simple 2-node conductive link, `hottest()` picks the
  right node.
- `test/ros/test_thermal_network_node.cpp` — gtest, links
  `thermal_network_ros`. Covers: node reaches `ACTIVE` after
  `maybe_autostart`'s delay without any external `ChangeState` call;
  `on_activate()`/`on_deactivate()` transitions succeed cleanly (mirrors
  `test_ars_node.cpp`).

`CMakeLists.txt` `BUILD_TESTING` block gets two `ament_add_gtest(...)`
entries analogous to ssos_eclss's.

## Also in scope: port `sun_vector` / `array_absorptivity` into `ssos_thermal`, Bullet-free

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

**Not yet planned:** removal of the orbit-calculation piece (the
Julian-date/ECI sun-position math in `sun_vector.hpp`, and/or GNC's
`orbit_dynamics` node) was mentioned as separate future work — no scope or
approach decided yet; revisit once that's ready to plan.

## Explicitly out of scope (this pass)

- [ ] `cooling_server`, `radiator`, `demand`, `array_absorptivity` stay as
      plain `rclcpp::Node`s, unregistered — no GUI/roster surface
      distinguishes them today.
- [ ] `main_window.py` → `_init_global_subs()` still only subscribes
      `("ars", "ogs", "wrs", "cabin")` heartbeats — adding `"thermal"` is a
      separate GUI follow-up.
- [ ] `space_station.launch.py` still doesn't launch
      `space_station_thermal_control` at all (same situation GNC was in
      before this session's earlier fix) — separate follow-up.
- [ ] `docs/architecture.md` for this package (mirroring
      `ssos_eclss/docs/architecture.md`) — worth adding once this lands,
      not blocking it.

## Verification checklist

- [ ] `pixi run build` (or `colcon build --packages-select space_station_thermal_control`) compiles clean
- [ ] `colcon test --packages-select space_station_thermal_control` passes (new unit + ros tests)
- [ ] `ros2 launch space_station_thermal_control thermals.launch.py`, then
      `ros2 lifecycle get /thermal_network` shows `active` within ~1s with
      **no manual lifecycle call** (confirms `maybe_autostart` works)
- [ ] `ros2 topic echo /ssos/thermal/heartbeat` shows periodic `healthy=true`, `lifecycle_state=2 (ACTIVE)`
- [ ] With `ssos_core`'s `system_manager` also running: no
      `"system_manager registration service unavailable"` warning in the
      thermal_nodes log
- [ ] `ros2 param set thermal_network enable_failure true` + lowered
      `max_temp_threshold` → exactly **one** `FaultEvent` on
      `/ssos/fault_event` at the transition (not one per tick)
- [ ] `/thermal/nodes/state` and `/thermal/links/flux` still publish
      identical data shape to before (regression check — the GUI's
      `ThermalWidget` must keep working unmodified)
