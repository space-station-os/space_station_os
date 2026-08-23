# Architecture

`ssos_thermal` follows the same physics/ROS split as `ssos_eclss`, sized for
two lifecycle-managed subsystems (`thermal_network`, `coolant_node`) plus
two small ROS-only utility nodes (`sun_vector_node`, `solar_heat_node`) that
were ported off Bullet Physics rather than converted to the physics/ROS
split.

```
+-------------------------------------------------------------+
| Layer 3: ROS (src/nodes, src/main)                          |
|   ThermalNetworkNode   CoolantNode   (both LifecycleNode)    |
|   ThermalDiagnostics (shared)                                |
|   SunVectorNode   SolarHeatNode        (plain rclcpp::Node)  |
+----------------------------+--------------------------------+
                             | (owns, drives step())
                             v
+-------------------------------------------------------------+
| Layer 2: thermal_network_physics (NO ROS)                   |
|   ThermalNetwork: load_from_yaml(), step(), hottest()       |
|   CoolantLoop: step()                                       |
+-------------------------------------------------------------+
```

`ThermalNetworkNode` and `CoolantNode` each have a physics object
underneath them — the RK4 node/link solver and the cooldown-loop model are
both genuinely reusable outside ROS (see [REFACTOR_PLAN.md](../REFACTOR_PLAN.md)
for why they were extracted, and for what was deliberately *not* ported
from the legacy `CoolantActionServer`: an inert Behavior-Tree tick loop,
two never-constructed service clients, an unused water-recycling method,
and two publishers that were declared but never published to).
`SunVectorNode`/`SolarHeatNode` are thin ROS nodes with no separate physics
object to extract; their only architectural change from the legacy
`sun_vector`/`array_absorptivity` executables is dropping the Bullet Physics
dependency in favor of [`math3d.hpp`](../include/ssos_thermal/math3d.hpp)'s
`Vector3`/`Quaternion` structs (see [ECLSS_PATTERN_REFERENCE.md](../ECLSS_PATTERN_REFERENCE.md)
for the pattern this whole package mirrors).

`ThermalDiagnostics` is shared by both lifecycle nodes — `subsystem_name`
is passed as a parameter to `make_heartbeat`/`make_fault` (mirroring
`EclssDiagnostics`) rather than baked in, now that there's more than one
caller (`"thermal"` and `"coolant"`).

## The core principle: physics has zero ROS dependencies

`thermal_network_physics` links nothing from ROS — not even `yaml-cpp`'s
consumer, `ament_index_cpp`, is linked into it; only the ROS node layer
resolves the config file's on-disk *path*, then hands the raw path string to
`ThermalNetwork::load_from_yaml()`. The same solver could run standalone or
in a flight controller unchanged.

`thermal_network_ros` depends on `thermal_network_physics` plus `rclcpp`,
`rclcpp_lifecycle`, `rclcpp_action` (for the coolant-loop action client) and
`space_station_interfaces`.

## Data flow

```
ThermalNetworkNode --/ssos/thermal/heartbeat--> system_manager
ThermalNetworkNode --/ssos/fault_event-->        system_manager
ThermalNetworkNode --/ssos/register_subsystem--> system_manager (on activate)
ThermalNetworkNode --/thermal/nodes/state------>  telemetry consumers (GUI)
ThermalNetworkNode --/thermal/links/flux------->  telemetry consumers (GUI)
ThermalNetworkNode --/thermals/diagnostics----->  telemetry consumers
ThermalNetworkNode <--/coolant_heat_transfer----  CoolantNode (action, in-package)

CoolantNode --/ssos/coolant/heartbeat--> system_manager
CoolantNode --/ssos/register_subsystem--> system_manager (on activate)
CoolantNode --/coolant_heat_transfer (feedback)--> ThermalNetworkNode, GUI's ThermalWidget
CoolantNode <--/tcs/radiator_a/vent_heat--------  radiator (legacy pkg, best-effort service)

SunVectorNode  --/sun_vector_body--> SolarHeatNode --/thermal/solar_heat--> (consumers)
SunVectorNode  <--/gnc/pose_all----  GNC (spacecraft pose)
```

`ThermalNetworkNode` does not subscribe to `/sim/world_state` — unlike
`ssos_eclss`'s nodes, the thermal network has no cabin-conditions input
today; its heat sources are the per-node `internal_power` values from
`config/thermal_nodes.yaml`. `subscribed_topics` in its
`RegisterSubsystem` request is therefore empty.

`radiator` stays in the legacy `space_station_thermal_control` package
(out of scope for this migration — see REFACTOR_PLAN.md); `CoolantNode`'s
`VentHeat` client talks to it by service name only and degrades gracefully
(logs an error, doesn't block) if it isn't running, so neither package
needs to depend on the other.

**GUI timing note:** the mission-control GUI's `ThermalWidget` sends a
one-shot test goal to `/coolant_heat_transfer` to populate its
Internal/Ammonia Temp cards. Since `CoolantNode` self-activates on its own
`autostart_delay_ms` (11s in the full-station launch) which can be well
after the GUI widget is constructed, `space_station/space_station/thermal.py`
polls `ActionClient.server_is_ready()` from its existing 1 Hz update timer
rather than doing one blocking `wait_for_server()` at startup -- a
blocking check with a short timeout would give up long before `CoolantNode`
comes up and leave those cards on "NO DATA" forever.

## Stepping model

`ThermalNetworkNode::updateSimulation()` runs on a wall timer at
`thermal_update_dt` Hz: it checks `coolingCallback()` (sends a `Coolant`
action goal once average temperature crosses `cooling_trigger_threshold`),
steps `ThermalNetwork` by `dt` unless a cooling goal is in flight, publishes
node/link telemetry and a `DiagnosticStatus`, then derives `healthy` from
`enable_failure && hottest().temperature > max_temp_threshold` and publishes
a heartbeat every tick plus a fault on the healthy→unhealthy edge only (via
`ThermalDiagnostics::should_raise_fault`).

`CoolantNode` is goal-driven rather than timer-driven: `CoolantLoop::step()`
only runs inside the action's `execute()` loop, once per accepted `Coolant`
goal, moving the incoming temperature at most 2.5 degC per ~100ms iteration
toward `target_temp_c` and publishing feedback each step, until it settles
within 0.5 degC of the target. A separate 1 Hz `heartbeat_timer_` publishes
`/ssos/coolant/heartbeat` independent of whether a goal is in flight
(always `healthy=true` today -- no fault condition is modeled for the
coolant loop yet).

## Lifecycle

`ThermalNetworkNode` and `CoolantNode` both self-activate via
`ThermalDiagnostics::maybe_autostart` (configures then activates itself
`autostart_delay_ms` after construction, gated on the `autostart`
parameter) so `launch/thermal.launch.py` doesn't need to emit lifecycle
`ChangeState` events — the same mechanism `ssos_eclss/launch/eclss.launch.py`
uses for its five nodes.
