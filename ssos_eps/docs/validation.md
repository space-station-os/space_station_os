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
| `test_eps_distribution` | default topology, all loads served, load demand driving channels and batteries, request admission, multiple loads within DDCU ratings, invalid requests, MBSU supply-margin rejection, release, DDCU overload shedding by priority, station overload and recovery, MBSU shortfall shedding, channel loss rerouting → redundancy lost → unstable, DDCU and MBSU faults, distribution energy balance, array/battery degradation flags, SOC override keeping failed ORUs |
| `test_fault_injection` | simulation fault descriptions vs EPS's own events, flat JSON params, expiry, per-unit keys, clear one / clear all, defaults, one-shot SOC, unknown types |
| `test_eps_orbit` | 12 × 2 topology, sunlight/eclipse modes, SOC cycling over three orbits without load shedding, per-step bus balance, eclipse energy conservation, overload → low SOC + unserved load, degradation, gimbal lock, BCDU fault, ORU failure and ORU over-temperature, live parameter update |
| `test_eps_node` | lifecycle, frozen sim time, BCDU / BatteryState / heartbeat telemetry, world-state eclipse switching, single low-SOC fault, ORU over-temperature heartbeat + fault event, live fault hook, parameter rejection, load-request service (request, release, invalid), MBSU/DDCU status telemetry, DDCU fault hook + single fault event, simulation-injected faults (applied, expired, ignored when out of range or self-published), stacking with parameters and clear_faults, load status and heat-load publishing, cold-plate temperature from thermal, thermal coupling disabled |
| `test_eps_scenario` | `system_manager` + `simulation_controller` + `eps_node` running scenario files: BCDU fault and load spike each go NOMINAL → DEGRADED → RECOVERY → NOMINAL, with the injected and detected fault events |
| `test_station_integration` | the whole station in one process: `system_manager`, `simulation_controller`, `eps_node`, ARS/OGS/WRS/Sabatier on EPS power, `thermal_network` on the station network. ECLSS loads registered and served, EPS heat on the cold plate and cabin nodes, cold-plate temperature read back, a load spike shedding OGS (O2 to zero, DEGRADED) and full recovery to NOMINAL. See [integration.md](integration.md) |
| `test_eps_parameter_bridge` | defaults round-trip, overrides, range and cross-parameter validation, topology and baseline loads, static classification |

## Orbit behaviour (defaults, 30 kW load)

`eps_orbit_validation 10 30000`:

```
orbit    gen_kWh   load_kWh  soc_min  soc_max  soc_end  unserved_Wh   beta_deg
    1       83.3       46.3   0.8000   0.9246   0.8780          0.0        0.0
    2       77.8       46.3   0.8748   0.9632   0.9169          0.0        0.5
    3       77.8       46.3   0.9137   0.9794   0.9333          0.0        1.0
    4       77.8       46.3   0.9301   0.9888   0.9428          0.0        1.5
    5       77.8       46.3   0.9396   0.9937   0.9478          0.0        2.0
   10       77.8       46.3   0.9469   0.9960   0.9501          0.0        4.6
```

The daylight surplus refills the bank each orbit and the SOC settles into a
steady cycle (about 0.95 to 1.0) with nothing shed. The beta gimbals track β,
so β only matters near the peak of its ~60 day cycle, where |β| above the lock
limit derates the array by `gimbal_lock_derate`.

The legacy controller instead cycled β every 10 orbits with a cos(β) derate,
which at β = 71° cut the array to about 27 kW, below the ~31.7 kW of load plus
DDCU losses, so the bank drained within a few hours of sim time. That behaviour
is still available with `solar.array.beta_precession_orbits: 10` and
`solar.array.derate_with_beta_cosine: true`.

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
- β follows a sinusoidal precession on orbit count rather than an orbital
  geometry model; `/sim/world_state` does not carry β, and the eclipse length
  from `ssos_sim` does not shorten at high β.
- Battery OCV has no temperature or hysteresis dependence; no RC dynamics.
