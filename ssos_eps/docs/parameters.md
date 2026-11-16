# Parameters

Every physical parameter is a ROS 2 parameter on `eps_node`, mapped through the
single `EpsParameterBridge`. The physics library never hardcodes values —
factory functions supply defaults only. Names mirror the physics structs.

| File | Namespace |
|------|-----------|
| `config/eps_parameters.yaml` | node-level, `eps.*`, `faults.*` |
| `config/solar_parameters.yaml` | `solar.array.*`, `solar.thermal.*`, `solar.ssu.*` |
| `config/battery_parameters.yaml` | `battery.*` (one ORU; all ORUs share it) |
| `config/bcdu_parameters.yaml` | `bcdu.*` (one channel; all channels share it) |
| `config/mbsu_parameters.yaml` | `mbsu.*` (all MBSUs share it) |
| `config/ddcu_parameters.yaml` | `ddcu.*` (all DDCUs share it) |
| `config/loads_parameters.yaml` | `loads.*` (baseline loads, demand scale) |

## Static vs dynamic

- **Static** — read in `on_configure`, need a reconfigure cycle:
  `eps.n_channels`, `eps.orus_per_channel`, `battery.n_series_cells`,
  `battery.capacity_ah`, `battery.soc_points`, `battery.cell_ocv_v`,
  `battery.initial_soc`, `battery.initial_temp_c`, `eps.n_mbsu`,
  `eps.channel_mbsu`, `eps.ddcu_mbsu`, `loads.ids`, `loads.power_w`,
  `loads.priority`, `loads.ddcu`.
- **Dynamic** — everything else; validated in the set-parameters callback and
  applied on the next step without resetting battery state.

## Node-level

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `step_rate_hz` | 1.0 | physics step rate (wall) |
| `enable_auto_faults` | true | publish `FaultEvent`s on health transitions |
| `faults.failed_oru` | -1 | index of an ORU to fail (-1 = none) |
| `faults.faulted_ddcu` | -1 | DDCU to trip (-1 = none) |
| `faults.faulted_mbsu` | -1 | MBSU to isolate (-1 = none) |
| `faults.offline_channel` | -1 | channel whose MBSU feeder is open (-1 = none) |
| `faults.faulted_bcdu` | -1 | channel whose BCDU is faulted (-1 = none) |
| `eps.low_soc_threshold` | 0.20 | bank SOC below which EPS is unhealthy |

## Solar

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `solar.array.area_m2` | 375 | total active cell area [m²] |
| `solar.array.cell_efficiency` | 0.165 | BOL efficiency (≈84 kW at 1361 W/m²) |
| `solar.array.degradation_factor` | 1.0 | fraction of BOL output |
| `solar.array.sunrise_ramp_s` | 480 | half-cosine re-acquisition after eclipse exit [s] |
| `solar.array.derate_with_beta_cosine` | true | apply cos(β) |
| `solar.array.beta_max_deg` | 75 | peak β over a precession cycle |
| `solar.array.beta_precession_orbits` | 10 | orbits per β cycle (counted on sunrise) |
| `solar.array.beta_lock_limit_deg` | 72 | \|β\| at which the SARJ locks |
| `solar.array.gimbal_lock_derate` | 0.6 | extra factor while locked |
| `solar.array.use_fixed_beta` / `fixed_beta_deg` | false / 0 | hold β constant |
| `solar.thermal.ambient_temp_c` | 20 | panel temperature with no generation [°C] |
| `solar.thermal.temp_rise_per_kw_c` | 0.4 | steady-state rise [°C/kW] |
| `solar.thermal.time_constant_s` | 300 | first-order time constant [s] |
| `solar.ssu.voltage_setpoint_v` | 160 | regulated bus voltage [V] |
| `solar.ssu.voltage_ripple_v` / `ripple_period_s` | 0.6 / 1350 | regulation ripple |
| `solar.ssu.efficiency` | 0.985 | SSU conversion efficiency |
| `solar.ssu.min_input_power_w` | 1 | dropout threshold [W] |

## Battery ORU

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `battery.n_series_cells` | 30 | cells in series |
| `battery.capacity_ah` | 134 | rated capacity [Ah] |
| `battery.state_of_health` | 1.0 | usable fraction of capacity |
| `battery.soc_points` / `cell_ocv_v` | NMC curve, 3.00–4.10 V | OCV table per cell |
| `battery.internal_resistance_ohm` | 0.05 | pack series resistance [Ω] |
| `battery.coulombic_efficiency` | 0.99 | charge accepted / supplied |
| `battery.max_charge_current_a` | 40 | per-ORU charge limit [A] |
| `battery.max_discharge_current_a` | 100 | per-ORU discharge limit [A] |
| `battery.max_cell_voltage_v` | 4.10 | CV charge limit per cell [V] |
| `battery.min_cell_voltage_v` | 3.00 | discharge cut-off per cell [V] |
| `battery.thermal_mass_j_k` | 2.0e5 | lumped heat capacity [J/K] |
| `battery.heat_transfer_w_k` | 20 | conductance to cold plate [W/K] |
| `battery.coldplate_temp_c` | 10 | cold plate temperature [°C] |
| `battery.max_temp_c` | 35 | over-temperature limit [°C] |
| `battery.initial_soc` / `initial_temp_c` | 0.80 / 10 | state at configure |

## BCDU

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `bcdu.charge_enter_v` | 158 | SSU voltage at/above which the array is a source [V] |
| `bcdu.discharge_enter_v` | 152 | SSU voltage at/below which it is lost [V] |
| `bcdu.regulation_voltage_v` | 150 | bus voltage held while discharging [V] |
| `bcdu.regulation_min_v` / `regulation_max_v` | 130 / 180 | setpoint limits [V] |
| `bcdu.max_charge_current_a` | 65 | bus-side charge limit per channel [A] |
| `bcdu.max_discharge_current_a` | 127 | bus-side discharge limit per channel [A] |
| `bcdu.charge_efficiency` / `discharge_efficiency` | 0.95 / 0.95 | conversion efficiency |
| `bcdu.deadband_w` | 10 | imbalance ignored before switching mode [W] |
| `bcdu.unserved_tolerance_w` | 1 | unmet load tolerated before degraded [W] |

The legacy controller switched to charge at 160 V, exactly the SSU setpoint,
so its ripple made the mode chatter; 158 V keeps the 152 V lower threshold and
clears the regulated band.

## Topology

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `eps.n_mbsu` | 4 | MBSUs |
| `eps.channel_mbsu` | `[0,0,0,1,1,1,2,2,2,3,3,3]` | MBSU fed by each channel (-1 = unconnected) |
| `eps.ddcu_mbsu` | `[0,0,1,1,2,2,3,3]` | MBSU feeding each DDCU; its length is the DDCU count |

## MBSU

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mbsu.min_source_voltage_v` / `max_source_voltage_v` | 120 / 180 | usable channel voltage window [V] |
| `mbsu.min_sources` | 1 | sources needed for a usable bus |
| `mbsu.redundant_sources` | 2 | sources below which redundancy is reported lost |

## DDCU

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `ddcu.input_min_v` / `input_max_v` | 115 / 173 | primary input window [V] |
| `ddcu.output_voltage_v` | 124.5 | no-load secondary voltage [V] |
| `ddcu.regulation_droop_v` | 1.5 | output drop at rated power [V] |
| `ddcu.rated_power_w` | 6250 | continuous output rating [W] |
| `ddcu.efficiency` | 0.96 | conversion efficiency |
| `ddcu.no_load_loss_w` | 50 | standby loss while regulating [W] |
| `ddcu.thermal_mass_j_k` | 2.0e4 | lumped heat capacity [J/K] |
| `ddcu.heat_transfer_w_k` | 8 | conductance to cold plate [W/K] |
| `ddcu.coldplate_temp_c` | 18 | cold plate temperature [°C] |
| `ddcu.trip_temp_c` / `reset_temp_c` | 70 / 55 | over-temperature trip and reset [°C] |
| `ddcu.initial_temp_c` | 20 | temperature at configure [°C] |

## Loads

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `loads.ids` | 10 station loads | load names |
| `loads.power_w` | 30 kW in total | nominal power per load [W] |
| `loads.priority` | 0–3 | 0 = vehicle-critical … 3 = payloads |
| `loads.ddcu` | all -1 | assigned DDCU (-1 = most headroom) |
| `loads.demand_scale` | 1.0 | multiplier on every load (live) |

## Validation rules

The set-parameters callback rejects values before they reach the physics:

| Parameter | Constraint |
|-----------|-----------|
| efficiencies (`*.cell_efficiency`, `solar.ssu.efficiency`, `bcdu.*_efficiency`, `battery.coulombic_efficiency`, `battery.state_of_health`) | ∈ (0, 1] |
| `degradation_factor`, `gimbal_lock_derate`, `initial_soc`, `eps.low_soc_threshold` | ∈ [0, 1] |
| `beta_max_deg`, `beta_lock_limit_deg` | ∈ [0, 90]; `fixed_beta_deg` ∈ [-90, 90] |
| capacities, voltages, thermal mass | > 0 |
| areas, currents, resistances, times, powers | ≥ 0 |
| `eps.n_channels`, `eps.orus_per_channel`, `battery.n_series_cells` | ≥ 1 |
| `bcdu.discharge_enter_v` | < `bcdu.charge_enter_v` |
| `bcdu.regulation_voltage_v` | ∈ [`regulation_min_v`, `regulation_max_v`] |
| `battery.min_cell_voltage_v` | < `battery.max_cell_voltage_v` |
| `battery.soc_points` / `cell_ocv_v` | same length ≥ 2; SOC strictly increasing in [0, 1]; OCV positive, non-decreasing |
| `ddcu.efficiency` | ∈ (0, 1]; `ddcu.input_min_v` < `ddcu.input_max_v`; `ddcu.reset_temp_c` < `ddcu.trip_temp_c` |
| `mbsu.min_source_voltage_v` | < `mbsu.max_source_voltage_v` |
| `eps.channel_mbsu` | one entry per channel, each -1 or < `eps.n_mbsu`; `eps.ddcu_mbsu` non-empty, entries -1 or < `eps.n_mbsu` |
| `loads.*` | same length; ids unique and non-empty; power > 0; priority ≥ 0; ddcu -1 or a DDCU index |
| `loads.demand_scale` | ∈ [0, 10]; `step_rate_hz` > 0 |

Cross-parameter rules compare against the current value of the other parameter
unless both are set in the same request.
