# Architecture

`ssos_thermal` splits physics from ROS, same as `ssos_eclss` (see
[ECLSS_PATTERN_REFERENCE.md](../ECLSS_PATTERN_REFERENCE.md)):

```
Layer 3 — ROS (src/nodes, src/main)
  ThermalNetworkNode   CoolantNode        (LifecycleNode)
        │ owns, calls step()
        ▼
Layer 2 — thermal_network_physics (NO ROS)
  ThermalNetwork::step()      RK4 conduction solver
  CoolantLoop::step()         cooldown model
```

`thermal_network_physics` links nothing from ROS, so the same solver code
could run standalone or on flight hardware. Only `thermal_network_ros`
(Layer 3) depends on `rclcpp`/`rclcpp_lifecycle`/`rclcpp_action`/
`space_station_interfaces`.

## Nodes

| Node | Type | Role |
|---|---|---|
| `thermal_network` | `LifecycleNode` | Steps the node/link conduction model, triggers cooling |
| `coolant_node` | `LifecycleNode` | Serves the `/coolant_heat_transfer` cooldown action |

## Thermal network model

`config/thermal_nodes.yaml` defines a 3-node star:

```
                 SolarPanel1
                (C=1200 J/°C, P=10 W)
                       │ k=0.5 W/°C
                       ▼
   base_link (C=30300 J/°C, P=1380 W)
                       ▲
                       │ k=0.5 W/°C
                (C=1200 J/°C, P=10 W)
                 SolarPanel2
```

Each node $i$ obeys a lumped-mass energy balance — internal heat
generation plus conduction to its linked neighbors, no radiation or solar
input yet:

$$
C_i \frac{dT_i}{dt} = P_i + \sum_{j} k_{ij} \left( T_j - T_i \right)
$$

| Symbol | Meaning |
|---|---|
| $T_i$ | node temperature [°C] |
| $C_i$ | `heat_capacity` [J/°C] |
| $P_i$ | `internal_power` [W], constant |
| $k_{ij}$ | `conductance` [W/°C] of the link between $i$ and $j$ |

`ThermalNetwork::step(dt)` integrates all three ODEs together with
classic 4th-order Runge-Kutta, at `thermal_update_dt` Hz. A node only
exchanges heat over a link whose other end is itself a declared node —
`base_link` has no `parent_link` (it's the root), so it gets no link of
its own; `SolarPanel1`/`SolarPanel2` each link to it.

There is no heat sink in this model (no radiation to space): total energy
only rises until the coolant loop intervenes (below). `base_link`'s
`heat_capacity`/`internal_power` are the *sums* of the ~46 individual
equipment nodes this package used to model separately — the aggregate
mass/power budget is unchanged, only the graph is simpler now. See
[REFACTOR_PLAN.md](../REFACTOR_PLAN.md) for the reduction and a dead-link
bug it fixed.

## Coolant loop model

`ThermalNetworkNode` sends a `Coolant` action goal to `coolant_node`
once `average_temperature() > cooling_trigger_threshold`. `CoolantLoop`
then runs this per-iteration model (`~100ms` per step, from `coolant.yaml`
defaults) until the node settles within `0.5°C` of `target_temp_c`:

$$
\begin{aligned}
\Delta T &= \min\left(2.5,\ T_{in} - T_{target}\right) && \text{[°C, capped per step]} \\
Q &= \frac{m \cdot c_p \cdot \Delta T}{1000} && \text{[kJ removed from the loop]} \\
Q_{NH_3} &= Q \cdot \eta && \text{[kJ transferred to ammonia]} \\
T_{NH_3} &= 5.0 + \frac{Q_{NH_3}}{1000} && \text{[°C, simplified/unvalidated]} \\
\text{vent} &= Q_{NH_3} \geq Q_{vent} \\
T_{out} &= T_{in} - \Delta T
\end{aligned}
$$

| Symbol | Meaning |
|---|---|
| $m$ | `mass_kg` — internal coolant loop water mass [kg] |
| $c_p$ | `specific_heat_j_per_kg_c` [J/(kg·°C)] |
| $\eta$ | `heat_transfer_efficiency` — fraction of removed heat transferred to ammonia |
| $Q_{vent}$ | `vent_threshold_kj` — ammonia heat above which a radiator vent is requested |

While a cooling goal is in flight, `ThermalNetworkNode` pauses its own
`step()` and instead snaps **all** node temperatures to each feedback
value (`set_all_temperatures()`) — this cooldown loop is the only heat
sink anywhere in the package.

## Data flow

```
thermal_network --/ssos/thermal/heartbeat, /ssos/fault_event, /ssos/register_subsystem-->  system_manager
thermal_network --/thermal/nodes/state, /thermal/links/flux, /thermals/diagnostics------->  GUI / telemetry
thermal_network <--/coolant_heat_transfer (action)----------------------------------------  coolant_node

coolant_node --/ssos/coolant/heartbeat, /ssos/register_subsystem-->  system_manager
coolant_node --/coolant_heat_transfer (feedback)------------------>  thermal_network, GUI ThermalWidget
coolant_node <--/tcs/radiator_a/vent_heat (best-effort service)----  radiator (legacy pkg)
```

`thermal_network` does not subscribe to `/sim/world_state` — its only
heat sources are the YAML `internal_power` values. `radiator` stays in
legacy `space_station_thermal_control`; `coolant_node`'s vent call
degrades gracefully if it isn't running.

## Lifecycle

`thermal_network` and `coolant_node` both self-activate via
`ThermalDiagnostics::maybe_autostart` (configure then activate
`autostart_delay_ms` after construction) — `launch/thermal.launch.py`
never sends a lifecycle `ChangeState`, same mechanism as
`ssos_eclss/launch/eclss.launch.py`.

## Further reading

- [parameters.md](parameters.md) — every tunable parameter, per node
- [fault_catalog.md](fault_catalog.md) — the one fault type, edge-triggered
- [commands.md](commands.md) — build/launch/inspect/test commands
- [REFACTOR_PLAN.md](../REFACTOR_PLAN.md) — history: why each design
  decision was made, what was intentionally left out of each port
