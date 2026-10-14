# Fault catalog

`ssos_thermal` currently defines one fault type, raised by
`ThermalNetworkNode::updateSimulation()`. Unlike `ssos_eclss`'s
`FaultInjector` registry (physics-level, schedulable via YAML), this is a
direct threshold check — there's no fault-injection framework in this
package yet.

`CoolantNode` defines none: it always reports `healthy=true` on
`/ssos/coolant/heartbeat`. Its most likely real fault condition -- venting
triggered but the legacy `radiator`'s `VentHeat` service unavailable -- is
only logged (`RCLCPP_ERROR`) today, not surfaced as a `FaultEvent`.

| Type | Severity | Trigger | `description` |
|------|----------|---------|----------------|
| `thermal_node_overheat` | `SEVERITY_CRITICAL` | `enable_failure == true` AND hottest node's temperature > `max_temp_threshold` | `"overheating: <hottest_node_name>"` |

`affected_interfaces` is always `["/thermal/nodes/state"]`.

## Edge-triggered, not level-triggered

`ThermalDiagnostics::should_raise_fault(healthy)` only returns true on the
healthy→unhealthy transition (tracked via an internal `was_healthy_` bool
inside the `ThermalDiagnostics` instance owned by the node) — a single
`FaultEvent` publishes when the hottest node first crosses the threshold,
not once per tick for as long as it stays hot. The heartbeat's `healthy`
field, by contrast, reflects current state every tick.

## Trying it

```bash
ros2 param set /thermal_network enable_failure true
ros2 param set /thermal_network max_temp_threshold 25.0
ros2 topic echo /ssos/fault_event      # one FaultEvent at the transition
ros2 topic echo /ssos/thermal/heartbeat  # healthy=false every tick until it cools
```
