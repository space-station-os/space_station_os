# ssos_eps

Physics-based simulation of the space station Electrical Power System (EPS)
for Space Station OS (SSOS) v0.9: solar generation, battery storage, the
battery charge/discharge units (BCDUs), and distribution through the main bus
switching units (MBSUs) and DC-to-DC converter units (DDCUs) to named station
loads.

This is a parallel package to the legacy `space_station_eps` (which it does not
touch). Like `ssos_eclss`, it keeps a **strict separation between physics and
ROS**: everything under `src/{common,solar,battery,bcdu,mbsu,ddcu,loads,system}` compiles with
**zero ROS dependencies**. ROS lives only in `src/nodes/` and `src/main/`.

## Subsystems

| Subsystem | Model | Highlights |
|-----------|-------|-----------|
| **Solar array** | Flux × area × efficiency | Sunrise re-acquisition ramp, beta-angle precession (~60 day cycle, beta gimbal tracking, optional legacy cosine derate), SARJ gimbal lock, degradation factor, first-order panel temperature |
| **SSU** | Sequential shunt unit | 160 V regulation with ripple, conversion efficiency, dropout below minimum input, shunting of unused power |
| **Battery ORU** | Equivalent circuit Li-ion | OCV(SOC) table + series resistance, coulomb counting with coulombic efficiency, CC/CV charge limits, discharge cut-off, I²R heating to a cold plate |
| **BCDU** | One per power channel | SSU-voltage hysteresis, charge from surplus / discharge for deficit, bus-side current limits, conversion losses, IDLE / CHARGING / DISCHARGING / FAULT, degraded detection |
| **MBSU** | Main bus switching unit | Routes primary power from its source channels (voltage window, connected, able to supply), shares demand by channel capability so a lost channel is rerouted, redundancy and unstable-bus detection |
| **DDCU** | DC-to-DC converter unit | 115–173 V input window, 124.5 V output with droop, 6.25 kW rating, efficiency + standby loss, overload limiting, lumped thermal model with over-temperature trip and reset |
| **Loads** | Named load registry | Power, priority (0 = most critical) and DDCU per load; admission against DDCU headroom and MBSU supply margin; least critical loads shed first |
| **EpsSystem** | Generation → storage → distribution | 12 channels × 2 ORUs, 4 MBSUs × 3 channels, 8 DDCUs; each step routes DDCU demand through the MBSUs onto the channels and BCDUs/batteries |

The legacy `sarj_mock.cpp`, `battery_health.cpp`, `bcdu_device.cpp`,
`mbsu_distributor.cpp` and `ddcu_device.cpp` were used as the behavioural
reference (84 kW array, 160 V SSU, 152/160 V BCDU hysteresis, 24 ORUs on 12
channels, 120 V MBSU channel acceptance, 124.5 V DDCU output with a 115–173 V
input window).

## Build & test

```bash
cd ~/ssos_ws
colcon build --packages-select ssos_eps
colcon test --packages-select ssos_eps
colcon test-result --verbose
```

## Standalone validation (no ROS)

```bash
ros2 run ssos_eps eps_orbit_validation               # 3 orbits at 30 kW
ros2 run ssos_eps eps_orbit_validation 10 30000 eps.csv
```

Prints one line per orbit (generation, load, SOC swing, unserved energy, beta)
using the same eclipse geometry as the `ssos_sim` default world model.

## Run the ROS node

```bash
ros2 launch ssos_eps eps.launch.py          # eps_node only (self-activates)
ros2 launch ssos_eps eps_system.launch.py   # system_manager + simulation_controller + eps_node
```

`eps_node` is an `rclcpp_lifecycle::LifecycleNode`. It subscribes to
`/sim/world_state` (`solar_flux_w_m2`, `in_eclipse`), `/ssos/fault_event`
(simulation-injected faults) and `/thermal/nodes/state` (cold-plate
temperature), registers with the
`system_manager` via `/ssos/register_subsystem`, and publishes:

| Topic | Type |
|-------|------|
| `/ssos/eps/diagnostics` | `diagnostic_msgs/DiagnosticArray` (`eps`, `eps/solar`, `eps/bcdu`, `eps/battery`, `eps/mbsu`, `eps/ddcu`, `eps/loads`) |
| `/ssos/eps/heartbeat` | `space_station_interfaces/SubsystemHeartbeat` |
| `/ssos/fault_event` | `space_station_interfaces/FaultEvent` (on transition only) |
| `/ssos/eps/bcdu/status` | `space_station_interfaces/BCDUStatus` (station summary) |
| `/ssos/eps/bcdu/channel_<n>/status` | `space_station_interfaces/BCDUStatus` |
| `/ssos/eps/battery/oru_<n>/state` | `sensor_msgs/BatteryState` |
| `/ssos/eps/battery/soc` | `std_msgs/Float64` (bank SOC, 0-1) |
| `/ssos/eps/power_balance_w` | `std_msgs/Float64` |
| `/ssos/eps/solar/{array_power_w,ssu_voltage_v,ssu_power_w,ssu_current_a,panel_temperature_c,beta_deg}` | `std_msgs/Float64` |
| `/ssos/eps/mbsu/mbsu_<n>/status` | `space_station_interfaces/MBSUStatus` |
| `/ssos/eps/ddcu/ddcu_<n>/status` | `space_station_interfaces/DDCUStatus` |
| `/ssos/eps/loads` | `space_station_interfaces/LoadStatusArray` (every load: priority, DDCU, requested, served, shed) |
| `/thermal/heat_loads` | `space_station_interfaces/HeatLoadArray` (source `eps`; when `thermal.enable_coupling`) |

`BCDUStatus.current_draw` is positive when the BCDU discharges onto the bus.
`BatteryState.current` follows the sensor_msgs convention (positive while
charging).

## Load interface

Downstream subsystems request power through the
`/ssos/eps/load_request` service (`space_station_interfaces/srv/Load`):

| Request field | Meaning |
|---------------|---------|
| `load_id` | unique load name; requesting an existing id updates it |
| `action` | `ACTION_REQUEST` (0) or `ACTION_RELEASE` (1) |
| `power_w` | requested power [W] |
| `priority` | 0 = most critical; higher numbers are shed first |
| `ddcu_index` | preferred DDCU, -1 = let EPS choose |

The response carries `success`, `message`, `granted_power_w`, `supply_voltage`
and the assigned `ddcu_index`. A request is rejected when the id is empty, the
power is not a positive finite number, the DDCU index does not exist, no
available DDCU has the headroom, or the MBSU feeding the DDCU cannot supply it.

```bash
ros2 service call /ssos/eps/load_request space_station_interfaces/srv/Load \
  "{load_id: experiment, action: 0, power_w: 1500.0, priority: 2, ddcu_index: -1}"
```

## Faults and scenarios

EPS faults can be injected through the SSOS simulation path:
`/sim/inject_fault` or a timed entry in an `ssos_sim` scenario file. Both
publish a `FaultEvent` for `eps` that `eps_node` applies to the physics, with an
optional duration in simulation seconds. EPS also publishes its own detected
faults, and `system_manager` recovers once EPS has been healthy for its
`recovery_hold_s`. See [docs/fault_catalog.md](docs/fault_catalog.md).

```bash
ros2 launch ssos_eps eps_system.launch.py scenario:=eps_bcdu_fault_recovery
ros2 service call /sim/inject_fault space_station_interfaces/srv/InjectFault \
  "{target_subsystem: eps, fault_type: load_spike, parameters_json: '{\"demand_scale\": 2.0}', duration_s: 120.0}"
```

## Station integration

The ECLSS units (ARS, OGS, WRS, Sabatier) request their own power from EPS and
lose function when shed; EPS heats the thermal network's cold plate and cabin
nodes and takes its cold-plate temperature from it. The whole loop is checked by
`test_station_integration`. See [docs/integration.md](docs/integration.md).

## Parameters

Every physical parameter is a ROS 2 parameter, mapped through the single
`EpsParameterBridge` and split across
`config/{eps,solar,battery,bcdu,mbsu,ddcu,loads}_parameters.yaml`.
See [docs/parameters.md](docs/parameters.md).

## Documentation

- [docs/architecture.md](docs/architecture.md) — layering, step order and data flow
- [docs/parameters.md](docs/parameters.md) — parameter reference
- [docs/fault_catalog.md](docs/fault_catalog.md) — injectable and detected faults, scenarios
- [docs/validation.md](docs/validation.md) — test coverage and orbit behaviour
- [docs/integration.md](docs/integration.md) — ECLSS power, thermal coupling, end-to-end test

## Relationship to space_station_eps

`ssos_eps` covers everything the legacy package did (SARJ-driven array, SSU,
battery health, BCDUs, MBSU distribution, DDCUs and the load service) in one
lifecycle node with a ROS-free physics library, and adds sim-time stepping,
fault injection through `ssos_sim`, `system_manager` supervision and the
ECLSS/thermal coupling. The legacy package is left in place and unchanged
apart from its DDCU client, which follows the replaced `Load.srv` fields;
`space_station.launch.py` runs `ssos_eps` only.
