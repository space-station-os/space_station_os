# Parameters

## `thermal_network` node

All dynamic; live-tunable via `ros2 param set` and applied on the next
`updateSimulation()` tick (`thermal_update_dt` additionally restarts the
step timer at the new period).

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `enable_failure` | `false` | Gate for overheat fault/heartbeat-unhealthy detection |
| `enable_cooling` | `true` | Gate for sending `Coolant` action goals |
| `cooling_trigger_threshold` | `330.0` | [°C] Average temperature above which a cooling goal is sent |
| `max_temp_threshold` | `420.0` | [°C] Hottest-node temperature above which the node reports unhealthy (only checked if `enable_failure`) |
| `cooling_rate` | `0.05` | Fractional rate of heat removed during cooling (currently unused by `updateSimulation`, kept for parity with the legacy solver) |
| `thermal_update_dt` | `0.5` | [s] Step timer period / RK4 integration step |
| `thermal_config_file` | `"config/thermal_nodes.yaml"` | Path (relative to the package share dir) to the node/link graph |

Plus the lifecycle-autostart parameters shared with every `ssos_eclss`-style
node: `autostart` (`false`), `autostart_delay_ms` (`300`) — see
[architecture.md](architecture.md#lifecycle).

Config lives in `config/thermal_network.yaml` (these parameters) and
`config/thermal_nodes.yaml` (the node/link graph: `node_name`,
`parent_link`, `heat_capacity`, `internal_power`, `conductance` per entry —
loaded by `ThermalNetwork::load_from_yaml`, unchanged shape from the legacy
solver).

### Tuning at runtime

```bash
# Accepted (dynamic):
ros2 param set /thermal_network max_temp_threshold 350.0

# Force an overheat fault for testing:
ros2 param set /thermal_network enable_failure true
ros2 param set /thermal_network max_temp_threshold 25.0
```

## `solar_heat_node`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `solar_constant` | `1361.0` | [W/m²] Solar irradiance at 1 AU |
| `panels_names` | `[]` | List of panel names; each name `<n>` requires `<n>.absorptivity`, `<n>.area`, `<n>.normal` (3-element list, body-frame unit vector) |

No default panel config is installed (none existed in the legacy
`array_absorptivity` executable either) — panels must be supplied via a
launch-time parameters file or `ros2 param set` before `/thermal/solar_heat`
publishes any entries.

## `sun_vector_node`

No declared parameters. Subscribes to `/gnc/pose_all`
(`geometry_msgs/PoseStamped`) and publishes `/sun_vector_body`
(`geometry_msgs/Vector3`) once a pose has been received.
