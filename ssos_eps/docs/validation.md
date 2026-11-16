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
| `test_mbsu` | demand shared by capability, lost channel rerouted, low-voltage source ignored, redundancy lost, unstable bus, shortfall, fault isolation |
| `test_ddcu` | efficiency and droop, legacy 115–173 V input window, overload limiting, brown-out, fault isolation, over-temperature trip and hysteretic reset, thermal equilibrium |
| `test_load_registry` | add/replace/remove, demand scale, per-DDCU totals, shed order (least critical, then largest) |
| `test_eps_distribution` | default topology, all loads served, load demand driving channels and batteries, request admission, multiple loads within DDCU ratings, invalid requests, MBSU supply-margin rejection, release, DDCU overload shedding by priority, station overload and recovery, MBSU shortfall shedding, channel loss rerouting → redundancy lost → unstable, DDCU and MBSU faults, distribution energy balance |
| `test_eps_orbit` | 12 × 2 topology, sunlight/eclipse modes, SOC cycling over three orbits without load shedding, per-step bus balance, eclipse energy conservation, overload → low SOC + unserved load, degradation, gimbal lock, BCDU fault, ORU failure and ORU over-temperature, live parameter update |
| `test_eps_node` | lifecycle, frozen sim time, BCDU / BatteryState / heartbeat telemetry, world-state eclipse switching, single low-SOC fault, ORU over-temperature heartbeat + fault event, live fault hook, parameter rejection, load-request service (request, release, invalid), MBSU/DDCU status telemetry, DDCU fault hook + single fault event |
| `test_eps_parameter_bridge` | defaults round-trip, overrides, range and cross-parameter validation, topology and baseline loads, static classification |

## Orbit behaviour (defaults, 30 kW load)

`eps_orbit_validation 10 30000`:

```
orbit    gen_kWh   load_kWh  soc_min  soc_max  soc_end  unserved_Wh   beta_deg
    1       83.3       46.3   0.8000   0.9246   0.8780          0.0        0.0
    2       55.9       46.3   0.8742   0.9329   0.8862          0.0       44.1
    3       24.9       46.3   0.8193   0.8862   0.8193          0.0       71.3
    4       24.9       46.3   0.7511   0.8192   0.7511          0.0       71.3
    5       55.9       46.3   0.7471   0.8092   0.7611          0.0       44.1
    6       77.8       46.3   0.7578   0.8369   0.7891          0.0        0.0
```

At low β the daylight surplus refills the bank each orbit. Near the β peak the
cos(β) derate (inherited from the legacy controller) cuts generation below the
orbit-average load and the batteries carry the deficit. The DDCUs add about
1.7 kW of conversion and standby losses on top of the 30 kW of load, so over a
full β cycle the bank ends a few percent lower than it started; no load is shed.

Doubling every load (`eps_orbit_validation 2 60000`) exceeds the DDCU ratings:
loads larger than a 6.25 kW DDCU are shed whole and the rest are shed by
priority, so about 31 kWh per orbit is delivered.

## Known simplifications

- Array power is shared evenly across channels; there is no per-wing array to
  channel mapping.
- One DDCU type and one MBSU parameter set for the whole station.
- Load admission uses the previous step's supply margin, so a request made
  just as conditions change (e.g. at eclipse entry) can be shed on the next
  step; shed loads come back automatically when there is room.
- One SSU for the whole array; no per-wing shunt state.
- β follows the legacy sinusoidal precession on orbit count rather than an
  orbital geometry model; `/sim/world_state` does not carry β.
- Battery OCV has no temperature or hysteresis dependence; no RC dynamics.
