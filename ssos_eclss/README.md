# ssos_eclss

High-fidelity, physics-based simulation of the International Space Station's
Environmental Control and Life Support System (ECLSS) for Space Station OS
(SSOS) v0.9.

This is a brand-new, parallel package to the existing `space_station_eclss`
(which it does not touch). Its defining architectural principle is a **strict
separation between physics and ROS**: everything under `src/{common,ars,ogs,wrs,
sabatier,cabin,faults}` compiles with **zero ROS dependencies**, so the same
physics code can run in simulation or on flight hardware. ROS lives only in
`src/nodes/` and `src/main/`.

## Subsystems

| Subsystem | Model | Highlights |
|-----------|-------|-----------|
| **ARS** | 4-Bed Molecular Sieve (4BCO2 EDU) | 1D finite-volume packed-bed PDE, Toth/competitive isotherms, precooler, blower, air-save pump, 10-60-10 cycle |
| **OGS** | PEM water electrolysis | Nernst + activation/ohmic/concentration overpotentials, stack thermal model |
| **WRS** | Urine + water processor | Vapor-compression distillation, multifiltration, catalytic oxidation |
| **Sabatier** | CO2 reduction | `CO2 + 4H2 -> CH4 + 2H2O`, Arrhenius kinetics — closes the loop |
| **Cabin** | Well-mixed atmosphere | O2/CO2/N2/H2O balances, crew metabolic loads, leak model |
| **Faults** | Physics-level injection | Sensor / actuator / thermal faults that alter real behaviour |

## Build & test

```bash
cd ~/ssos_ws
colcon build --packages-select ssos_eclss
colcon test --packages-select ssos_eclss
colcon test-result --verbose
```

## Standalone validation (no ROS)

```bash
ros2 run ssos_eclss ars_validation        # confirms 4.16-4.76 kg/day CO2 removal
ros2 run ssos_eclss parameter_sweep        # ppCO2 / flow / cycle / LTL sweeps
ros2 run ssos_eclss breakthrough_curve_gen # breakthrough curve CSV
```

## Run the ROS nodes

```bash
ros2 launch ssos_eclss eclss.launch.py       # all four subsystems
ros2 launch ssos_eclss ars_only.launch.py    # ARS only
```

Nodes are `rclcpp_lifecycle::LifecycleNode`s. They subscribe to
`/sim/world_state` (the Epic A boundary), register with the `system_manager`
via `/ssos/register_subsystem`, and publish telemetry, heartbeats on
`/ssos/<name>/heartbeat` and faults on `/ssos/fault_event`.

## Parameters

Every physical parameter is a ROS 2 parameter, mapped through the single
`EclssParameterBridge`. Static parameters (geometry, cell count) are read in
`on_configure`; dynamic parameters (heater power, cycle timing, flow, isotherm
what-if) are live-tunable and validated before they reach the physics. See
[docs/parameters.md](docs/parameters.md).

## Documentation

- [docs/architecture.md](docs/architecture.md) — layering and data flow
- [docs/ars_model.md](docs/ars_model.md) — 4BMS governing equations
- [docs/ogs_model.md](docs/ogs_model.md) — electrolysis model
- [docs/wrs_model.md](docs/wrs_model.md) — water recovery model
- [docs/parameters.md](docs/parameters.md) — parameter reference
- [docs/validation.md](docs/validation.md) — validation against ICES-2021-313
- [docs/fault_catalog.md](docs/fault_catalog.md) — fault taxonomy

## References

- **ARS:** Peters, Cmarik, Knox, "4BCO2 EDU Performance", ICES-2021-313 (2021);
  Knox, Cmarik, "CO2 Removal for the ISS — 4-Bed Molecular Sieve Material
  Selection and System Design" (2019).
- **OGS:** Takada et al., "Status of the Advanced Oxygen Generation Assembly",
  ICES-2023-311 (2023).
- **Sabatier:** Hintze, Meier, Shah, DeVor, "Sabatier System Design Study for a
  Mars ISRU Propellant Production Plant", ICES-2018-155 (2018).
- **WRS:** Williamson, Wilson, Robinson, Luong, "Status of ISS Water Management
  and Recovery", ICES-2023-097 (2023).
