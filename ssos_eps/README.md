# ssos_eps

Physics-based simulation of the space station Electrical Power System (EPS)
for Space Station OS (SSOS) v0.9: solar generation, battery storage and the
battery charge/discharge units (BCDUs).

This is a parallel package to the legacy `space_station_eps` (which it does not
touch). Like `ssos_eclss`, it keeps a **strict separation between physics and
ROS**: everything under `src/{common,solar,battery,bcdu,system}` compiles with
**zero ROS dependencies**. ROS lives only in `src/nodes/` and `src/main/`.

## Subsystems

| Subsystem | Model | Highlights |
|-----------|-------|-----------|
| **Solar array** | Flux × area × efficiency | Sunrise re-acquisition ramp, beta-angle precession and cosine derate, SARJ gimbal lock, degradation factor, first-order panel temperature |
| **SSU** | Sequential shunt unit | 160 V regulation with ripple, conversion efficiency, dropout below minimum input, shunting of unused power |
| **Battery ORU** | Equivalent circuit Li-ion | OCV(SOC) table + series resistance, coulomb counting with coulombic efficiency, CC/CV charge limits, discharge cut-off, I²R heating to a cold plate |
| **BCDU** | One per power channel | SSU-voltage hysteresis, charge from surplus / discharge for deficit, bus-side current limits, conversion losses, IDLE / CHARGING / DISCHARGING / FAULT, degraded detection |
| **EpsSystem** | Generation + storage | 12 channels × 2 ORUs, fixed step order array → SSU → BCDU → ORUs, station power balance and health |

The legacy `sarj_mock.cpp`, `battery_health.cpp` and `bcdu_device.cpp` were
used as the behavioural reference (84 kW array, 160 V SSU, 152/160 V BCDU
hysteresis, 24 ORUs on 12 channels). MBSU, DDCU and load distribution follow
in a separate package milestone; until then the station load is a single
configurable demand shared evenly across channels.

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
`/sim/world_state` (`solar_flux_w_m2`, `in_eclipse`), registers with the
`system_manager` via `/ssos/register_subsystem`, and publishes:

| Topic | Type |
|-------|------|
| `/ssos/eps/diagnostics` | `diagnostic_msgs/DiagnosticArray` (`eps`, `eps/solar`, `eps/bcdu`, `eps/battery`) |
| `/ssos/eps/heartbeat` | `space_station_interfaces/SubsystemHeartbeat` |
| `/ssos/fault_event` | `space_station_interfaces/FaultEvent` (on transition only) |
| `/ssos/eps/bcdu/status` | `space_station_interfaces/BCDUStatus` (station summary) |
| `/ssos/eps/bcdu/channel_<n>/status` | `space_station_interfaces/BCDUStatus` |
| `/ssos/eps/battery/oru_<n>/state` | `sensor_msgs/BatteryState` |
| `/ssos/eps/battery/soc` | `std_msgs/Float64` (bank SOC, 0-1) |
| `/ssos/eps/power_balance_w` | `std_msgs/Float64` |
| `/ssos/eps/solar/{array_power_w,ssu_voltage_v,ssu_power_w,ssu_current_a,panel_temperature_c,beta_deg}` | `std_msgs/Float64` |

`BCDUStatus.current_draw` is positive when the BCDU discharges onto the bus.
`BatteryState.current` follows the sensor_msgs convention (positive while
charging).

## Parameters

Every physical parameter is a ROS 2 parameter, mapped through the single
`EpsParameterBridge` and split across `config/{eps,solar,battery,bcdu}_parameters.yaml`.
See [docs/parameters.md](docs/parameters.md).

## Documentation

- [docs/architecture.md](docs/architecture.md) — layering, step order and data flow
- [docs/parameters.md](docs/parameters.md) — parameter reference
- [docs/validation.md](docs/validation.md) — test coverage and orbit behaviour
