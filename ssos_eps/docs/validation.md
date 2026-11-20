# Validation

## How to run

```bash
colcon test --packages-select ssos_eps && colcon test-result --verbose
ros2 run ssos_eps eps_orbit_validation 10 30000 eps.csv
```

## Test coverage

| Suite | Covers |
|-------|--------|
| `test_solar_array` | flux × area × efficiency, eclipse → 0, sunrise ramp shape, β cosine derate, gimbal lock, no lock at `beta_max_deg`, forced lock, β precession, degradation, panel thermal lag |
| `test_ssu` | regulation and efficiency, dropout, ripple band |
| `test_battery_oru` | OCV table, coulomb counting, coulombic efficiency, current limits, CV taper, SOC bounds, failure isolation, I²R heating, over-temperature, stored-energy integral, discharge energy conservation |
| `test_bcdu` | charge from surplus, discharge in eclipse and for daylight deficit, hysteresis, idle + shunt when full, bus charge limit, fault isolation, failed-ORU degradation, empty bank, current sharing, bus power balance |
| `test_mbsu` | demand shared by capability, lost channel rerouted, low-voltage source ignored, redundancy lost, unstable bus, shortfall, fault isolation |
| `test_ddcu` | efficiency and droop, legacy 115–173 V input window, overload limiting, brown-out, fault isolation, over-temperature trip and hysteretic reset, thermal equilibrium |
| `test_load_registry` | add/replace/remove, demand scale, per-DDCU totals, shed order (least critical, then largest) |
| `test_eps_distribution` | default topology, all loads served, load demand driving channels and batteries, request admission, multiple loads within DDCU ratings, invalid requests, MBSU supply-margin rejection, release, DDCU overload shedding by priority, station overload and recovery, MBSU shortfall shedding, channel loss rerouting → redundancy lost → unstable, DDCU and MBSU faults, distribution energy balance, array/battery degradation flags, SOC override keeping failed ORUs |
| `test_fault_injection` | simulation fault descriptions vs EPS's own events, flat JSON params, expiry, per-unit keys, clear one / clear all, defaults, low-SOC drill (SOC handed over once, drill tracked until expiry, dropped by clear_faults), overheat drills' per-unit heat, gimbal-lock drill, supported types equal to the simulation's EPS drill catalog, unknown types |
| `test_eps_orbit` | 12 × 2 topology, sunlight/eclipse modes, SOC cycling over three orbits without load shedding, per-step bus balance, eclipse energy conservation, overload → low SOC + unserved load, degradation, gimbal lock only by drill, settled start (SOC 0.95, DDCUs and ORUs at equilibrium on the first step, no DDCU drift), 16 orbits with no unhealthy step at precessing and fixed 5° β, ORU overheat drill → over-temperature → recovery, DDCU overheat drill → trip + shed → held tripped → recovery, BCDU fault, ORU failure and ORU over-temperature, live parameter update |
| `test_eps_node` | lifecycle, settled on-orbit start, every drill in the catalog injected with the simulation description format → unhealthy → exactly one detected fault event of any type (the root cause) → healthy again after `clear_faults` or expiry; low-SOC drill not handing the charge back when cleared; registration retried until `system_manager` accepts, frozen sim time, BCDU / BatteryState / heartbeat telemetry, world-state eclipse switching, single low-SOC fault, ORU over-temperature heartbeat + fault event, live fault hook, parameter rejection, load-request service (request, release, invalid), MBSU/DDCU status telemetry, DDCU fault hook + single fault event, simulation-injected faults (applied, expired, ignored when out of range or self-published), stacking with parameters and clear_faults, load status and heat-load publishing, cold-plate temperature from thermal, thermal coupling disabled |
| `test_eps_scenario` | `system_manager` + `simulation_controller` + `eps_node` running scenario files: BCDU fault and load spike each go NOMINAL → DEGRADED → RECOVERY → NOMINAL, with the injected and detected fault events |
| `test_station_integration` | the whole station in one process: `system_manager`, `simulation_controller`, `eps_node`, ARS/OGS/WRS/Sabatier on EPS power, `thermal_network` on the station network. ECLSS loads registered and served, EPS heat on the cold plate and cabin nodes, cold-plate temperature read back, a load spike shedding OGS (O2 to zero, DEGRADED) and full recovery to NOMINAL. See [integration.md](integration.md) |
| `test_eps_parameter_bridge` | defaults round-trip, overrides, range and cross-parameter validation, topology and baseline loads, static classification |

## Orbit behaviour (defaults, 30 kW load)

`eps_orbit_validation 10 30000`:

```
orbit  gen_kWh  load_kWh  soc_min  soc_max  soc_end  unserved_Wh  beta_deg ddcu_max_C oru_max_C  unhealthy_s
    1     83.3      46.3   0.9500   0.9977   0.9517          0.0       0.0       55.5      20.9            0
    2     77.8      46.3   0.9486   0.9963   0.9503          0.0       0.5       55.5      20.6            0
    3     77.8      46.3   0.9472   0.9960   0.9501          0.0       1.0       55.5      20.4            0
    4     77.8      46.3   0.9469   0.9960   0.9500          0.0       1.5       55.5      20.4            0
    5     77.8      46.3   0.9469   0.9960   0.9501          0.0       2.0       55.5      20.4            0
    6     77.8      46.3   0.9469   0.9960   0.9501          0.0       2.5       55.5      20.4            0
    7     77.8      46.3   0.9469   0.9960   0.9501          0.0       3.0       55.5      20.4            0
    8     77.8      46.3   0.9469   0.9960   0.9501          0.0       3.5       55.5      20.4            0
    9     77.8      46.3   0.9469   0.9960   0.9501          0.0       4.1       55.5      20.4            0
   10     77.8      46.3   0.9469   0.9960   0.9501          0.0       4.6       55.5      20.4            0
```

The station starts where it settles: the bank at 0.95 and every DDCU and ORU
at its thermal equilibrium, so orbit 1 already looks like orbit 10 and EPS
reports no unhealthy second. The hottest DDCU carries 6 kW and sits at 55.5 °C,
well under the 70 °C trip; the ORUs stay within about 1 °C of the 20 °C cold
plate. The same holds over 16 orbits (orbit 16: SOC 0.9468–0.9960, β 7.6°).

The daylight surplus refills the bank each orbit with nothing shed. The beta
gimbals track β over its whole ~60 day cycle: the lock limit (76°) is above
the peak β (75°), so the `gimbal_lock_derate` only applies in the
`array_gimbal_lock` drill.

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
