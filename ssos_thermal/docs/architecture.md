# Architecture

`ssos_thermal` follows the same physics/ROS split as `ssos_eclss`, sized for
one lifecycle-managed subsystem (`thermal_network`) plus two small
ROS-only utility nodes (`sun_vector_node`, `solar_heat_node`) that were
ported off Bullet Physics rather than converted to the physics/ROS split.

```
+-------------------------------------------------------------+
| Layer 3: ROS (src/nodes, src/main)                          |
|   ThermalNetworkNode (LifecycleNode)   ThermalDiagnostics    |
|   SunVectorNode   SolarHeatNode        (plain rclcpp::Node)  |
+----------------------------+--------------------------------+
                             | (owns, drives step())
                             v
+-------------------------------------------------------------+
| Layer 2: thermal_network_physics (NO ROS)                   |
|   ThermalNetwork: load_from_yaml(), step(), hottest()       |
+-------------------------------------------------------------+
```

Only `ThermalNetworkNode` has a physics layer underneath it — the RK4
node/link solver is genuinely reusable outside ROS (see
[REFACTOR_PLAN.md](../REFACTOR_PLAN.md) for why it was extracted).
`SunVectorNode`/`SolarHeatNode` are thin ROS nodes with no separate physics
object to extract; their only architectural change from the legacy
`sun_vector`/`array_absorptivity` executables is dropping the Bullet Physics
dependency in favor of [`math3d.hpp`](../include/ssos_thermal/math3d.hpp)'s
`Vector3`/`Quaternion` structs (see [ECLSS_PATTERN_REFERENCE.md](../ECLSS_PATTERN_REFERENCE.md)
for the pattern this whole package mirrors).

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
ThermalNetworkNode <--/coolant_heat_transfer----  cooling_server (legacy pkg, action)

SunVectorNode  --/sun_vector_body--> SolarHeatNode --/thermal/solar_heat--> (consumers)
SunVectorNode  <--/gnc/pose_all----  GNC (spacecraft pose)
```

`ThermalNetworkNode` does not subscribe to `/sim/world_state` — unlike
`ssos_eclss`'s nodes, the thermal network has no cabin-conditions input
today; its heat sources are the per-node `internal_power` values from
`config/thermal_nodes.yaml`. `subscribed_topics` in its
`RegisterSubsystem` request is therefore empty.

`cooling_server` stays in the legacy `space_station_thermal_control`
package (out of scope for this migration — see REFACTOR_PLAN.md); the
action client here talks to it by topic name only, so neither package needs
to depend on the other.

## Stepping model

`ThermalNetworkNode::updateSimulation()` runs on a wall timer at
`thermal_update_dt` Hz: it checks `coolingCallback()` (sends a `Coolant`
action goal once average temperature crosses `cooling_trigger_threshold`),
steps `ThermalNetwork` by `dt` unless a cooling goal is in flight, publishes
node/link telemetry and a `DiagnosticStatus`, then derives `healthy` from
`enable_failure && hottest().temperature > max_temp_threshold` and publishes
a heartbeat every tick plus a fault on the healthy→unhealthy edge only (via
`ThermalDiagnostics::should_raise_fault`).

## Lifecycle

`ThermalNetworkNode` self-activates via `ThermalDiagnostics::maybe_autostart`
(configures then activates itself `autostart_delay_ms` after construction,
gated on the `autostart` parameter) so `launch/thermal.launch.py` doesn't
need to emit lifecycle `ChangeState` events — the same mechanism
`ssos_eclss/launch/eclss.launch.py` uses for its five nodes.
