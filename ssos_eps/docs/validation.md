# Validation

## How to run

```bash
colcon test --packages-select ssos_eps && colcon test-result --verbose
ros2 run ssos_eps eps_orbit_validation 10 30000 eps.csv
```

## Test coverage

| Suite | Covers |
|-------|--------|
| `test_solar_array` | flux × area × efficiency, eclipse → 0, sunrise ramp shape, β cosine derate, gimbal lock, β precession, degradation, panel thermal lag |
| `test_ssu` | regulation and efficiency, dropout, ripple band |
| `test_battery_oru` | OCV table, coulomb counting, coulombic efficiency, current limits, CV taper, SOC bounds, failure isolation, I²R heating, over-temperature, stored-energy integral, discharge energy conservation |
| `test_bcdu` | charge from surplus, discharge in eclipse and for daylight deficit, hysteresis, idle + shunt when full, bus charge limit, fault isolation, failed-ORU degradation, empty bank, current sharing, bus power balance |
| `test_eps_orbit` | 12 × 2 topology, sunlight/eclipse modes, SOC cycling over three orbits without load shedding, per-step bus balance, eclipse energy conservation, overload → low SOC + unserved load, degradation, gimbal lock, BCDU fault and ORU failure, live parameter update |
| `test_eps_node` | lifecycle, frozen sim time, BCDU / BatteryState / heartbeat telemetry, world-state eclipse switching, single low-SOC fault, live fault hook, parameter rejection |
| `test_eps_parameter_bridge` | defaults round-trip, overrides, range and cross-parameter validation, static classification |

## Orbit behaviour (defaults, 30 kW load)

`eps_orbit_validation 10 30000`:

```
orbit    gen_kWh   load_kWh  soc_min  soc_max  soc_end  unserved_Wh   beta_deg
    1       83.3       46.3   0.8000   0.9286   0.8846          0.0        0.0
    2       55.9       46.3   0.8811   0.9460   0.9021          0.0       44.1
    3       24.9       46.3   0.8435   0.9021   0.8435          0.0       71.3
    4       24.9       46.3   0.7842   0.8435   0.7842          0.0       71.3
    5       55.9       46.3   0.7806   0.8468   0.8021          0.0       44.1
    6       77.8       46.3   0.7991   0.9171   0.8730          0.0        0.0
```

At low β the daylight surplus refills the bank each orbit. Near the β peak the
cos(β) derate (inherited from the legacy controller) cuts generation below the
orbit-average load and the batteries carry the deficit; the bank recovers as β
falls. No load is shed over the cycle at 30 kW.

## Known simplifications

- Array power and load are shared evenly across channels; MBSU/DDCU routing and
  per-load accounting are not modelled yet.
- One SSU for the whole array; no per-wing shunt state.
- β follows the legacy sinusoidal precession on orbit count rather than an
  orbital geometry model; `/sim/world_state` does not carry β.
- Battery OCV has no temperature or hysteresis dependence; no RC dynamics.
