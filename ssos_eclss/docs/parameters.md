# Parameters

Every physical parameter is a ROS 2 parameter mapped through the single
`EclssParameterBridge`. The physics library never hardcodes values — factory
functions supply defaults only, and parameters flow IN from ROS.

## Static vs dynamic

- **Static** (`ars.bed.*` — geometry, cell count): read once in `on_configure`.
  Changing them requires a reconfigure cycle to rebuild the beds.
- **Dynamic** (everything else — heater, cycle, flow, LTL temp, isotherm
  what-if, efficiency): live-tunable via the set-parameters callback and applied
  on the next step.

## EPS power

`eps_coupling` (default `true`, on ARS, OGS, WRS and Sabatier) makes the unit
request its power from `ssos_eps` and lose function when EPS sheds it. It is
read in `on_configure`. With it off, or with EPS not running, the unit behaves
as if fully powered. See `ssos_eps/docs/integration.md`.

## Station units

Defaults are in `config/*_parameters.yaml`; the values below set the on-orbit
starting state and the caution limits.

| Node | Parameter | Default | Meaning |
|------|-----------|---------|---------|
| crew | `crew_size` | 4 | crew aboard |
| crew | `initial_time_of_day_hr` | 10.75 | crew time at simulation start |
| crew | `sleep_start_hr`, `sleep_hours` | 21.5, 8 | shared sleep window |
| crew | `first_exercise_hr`, `exercise_spacing_hr` | 8, 2 | staggered exercise slots |
| crew | `metabolic_scale` | 1.0 | multiplier on gas, latent and heat loads |
| cabin | `cabin_volume_m3` | 388 | ISS habitable volume |
| cabin | `initial_co2_ppm`, `initial_relative_humidity` | 2600, 0.45 | starting air |
| cabin | `leak_area_m2` | 2e-8 | baseline leak, about 0.25 kg/day |
| cabin | `co2_alarm_ppm` | 5300 | about 4 mmHg ppCO2 |
| cabin | `o2_low_pct`, `o2_high_pct` | 19.5, 23.5 | O2 limits |
| cabin | `pressure_low_kpa`, `pressure_high_kpa` | 97, 104 | pressure limits |
| cabin | `rh_low`, `rh_high` | 0.25, 0.75 | humidity limits |
| cabin | `leak_alarm_kg_day` | 3.0 | leak rate alarm |
| ars | `warm_start_s` | 1200 (config), 0 (node) | bed pre-roll at configure, about 30 ms per 5 s |
| ars | `min_capture_efficiency` | 0.5 | alarm below this averaged single-pass capture |
| ars | `capture_time_constant_s` | 4800 | averaging time (about a half-cycle) |
| ogs | `stack_current_a` | 17.5 | starting current, about 3.45 kg O2/day |
| ogs | `o2_setpoint_kpa` | 21.0 | cabin ppO2 target |
| ogs | `o2_kp_a_per_kpa`, `o2_integral_time_s` | 20, 86400 | ppO2 PI gains |
| wrs | `initial_potable_kg`, `initial_wastewater_kg` | 400, 8 | starting inventory |
| wrs | `wastewater_capacity_kg`, `potable_capacity_kg` | 22, 1000 | tank sizes |
| wrs | `upa_start_fraction`, `upa_stop_fraction` | 0.70, 0.05 | UPA batch thresholds |
| wrs | `low_supply_days` | 7 | potable reserve warning |
| wrs | `wpa_reprocess_kg_day`, `wpa_waste_capacity_kg` | 10, 100 | held water fed back through the WPA, and its tank |
| sabatier | `min_reactor_temp_k` | 550 | reactor cold warning |

All units have `enable_auto_faults` (true): limit checks on. Drills are
reported whether or not it is set.

## Validation

The set-parameters callback rejects out-of-range values with a clear message
before they reach the physics:

| Parameter | Constraint |
|-----------|-----------|
| `ars.isotherm.co2_13x.t0` | Toth exponent ∈ (0, 1] |
| `ars.isotherm.co2_13x.{q_m0,b0,dH}` | > 0 |
| `ars.bed.*.voidage` | ∈ (0, 1) |
| `ars.efficiency.*` | ∈ [0, 1] |
| `ars.operating.*_temp_k`, `ars.heater.max_temp_k` | > 0 K |
| flows, powers, times, lengths, LDF rates | ≥ 0 |
| `ars.bed.*.n_cells` | ≥ 1 |

## ARS parameter namespace (example)

```
ars.bed.adsorbent.length          ars.bed.adsorbent.diameter
ars.bed.adsorbent.voidage         ars.bed.adsorbent.particle_diameter
ars.bed.adsorbent.n_cells         (and ars.bed.desiccant.*)
ars.isotherm.co2_13x.q_m0         ars.isotherm.co2_13x.b0
ars.isotherm.co2_13x.dH           ars.isotherm.co2_13x.t0
ars.ldf.adsorbent.k_co2           ars.ldf.adsorbent.k_h2o
ars.heater.total_power_w          ars.heater.central_power_w
ars.heater.max_temp_k
ars.cycle.air_save_s              ars.cycle.adsorb_s   ars.cycle.vacuum_s
ars.operating.inlet_flow_scfm     ars.operating.inlet_ppco2_torr
ars.operating.ltl_inlet_temp_k    ars.operating.ltl_flow_gpm
ars.operating.cabin_temp_k        ars.operating.cabin_pressure_pa
ars.operating.vacuum_pressure_pa
ars.efficiency.capture_efficiency ars.efficiency.holdup_loss
```

## Tuning at runtime

```bash
# Accepted (dynamic):
ros2 param set /ars_node ars.heater.total_power_w 800.0

# Rejected with a reason (out of range):
ros2 param set /ars_node ars.isotherm.co2_13x.t0 1.5
```

Config files live under `config/` and are installed to
`share/ssos_eclss/config/`. Nested YAML keys map to the dotted parameter names.
