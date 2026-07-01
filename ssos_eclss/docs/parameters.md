# Parameters

Every value is a ROS 2 parameter. Node operating parameters apply **live**
through each node's `on_set_parameters` handler; the ARS's full physics set is
mapped and validated through the single `EclssParameterBridge`. The physics
library never hardcodes values — factory functions supply defaults only, and
parameters flow in from ROS. The mission-control GUI's **PARAMS** editor lists
and sets all of them at runtime.

Config files live under `config/` (installed to `share/ssos_eclss/config/`);
nested YAML keys map to the dotted parameter names.

---

## 1. Common to every node

| Parameter | Default | Notes |
|-----------|---------|-------|
| `use_sim_time` | `true` | drive off `/clock` |
| `autostart` | `false` (launch sets `true`) | self-configure + activate |
| `autostart_delay_ms` | 300 | delay before self-activation |
| `step_rate_hz` | 1.0 (ARS 10.0) | node step rate |

---

## 2. Node operating parameters (live)

### crew_node

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `day_length_s` | 86400 | sim-seconds per diurnal cycle (lower = faster cycle; daily averages hold) |
| `metabolic_scale` | 1.0 | global multiplier on CO₂/O₂/latent (fitness/activity) |
| `crew_size` | 4 | number of crew |
| `drink_kg_day` | 2.20 | drinking + hygiene + food prep, per person (from potable bus) |
| `flush_kg_day` | 0.30 | toilet flush, per person (from potable bus) |
| `urine_kg_day` | 1.20 | urine to WRS, per person |
| `feces_water_kg_day` | 0.15 | faecal water (lost), per person |
| `trash_water_kg_day` | 0.20 | wet-trash water (lost), per person |

### cabin_node

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `cabin_volume_m3` | 100 | free cabin volume |
| `cabin_temp_c` | 22 | controlled cabin temperature |
| `co2_alarm_ppm` | 7000 | ppCO₂ alarm threshold (fault when `enable_auto_faults`) |
| `enable_auto_faults` | `false` | opt-in threshold faults |

### ogs_node

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `stack_current_a` | 27.0 | PSM current (10–46.9 A; 46.9 → 9.25 kg/day) |
| `o2_required_kg_day` | 2.3 | O₂ requirement (fault threshold) |
| `enable_auto_faults` | `false` | opt-in threshold faults |

### wrs_node

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `potable_limit_us` | 100 | potable conductivity limit [µS/cm] (fault threshold) |
| `wastewater_capacity_kg` | 22 | WSTA urine tank capacity |
| `potable_capacity_kg` | 1000 | potable tank capacity |
| `initial_potable_kg` | 300 | starting potable inventory |
| `max_urine_process_kg_day` | 13.6 | UPA throughput when running |
| `upa_start_fraction` | 0.70 | WSTA fill fraction that starts a batch |
| `upa_stop_fraction` | 0.05 | WSTA fraction that ends a batch |
| `potable_reserve_kg` | 5 | reserve below which OGS feedwater draw is starved |
| `enable_auto_faults` | `false` | opt-in threshold faults |

### ars_node (node-level; physics set is in §3)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `co2_required_kg_day` | 4.16 | removal requirement (fault threshold) |
| `enable_auto_faults` | `false` | opt-in threshold faults |

### sabatier_node

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `step_rate_hz` | 1.0 | node step rate (reactor thermal is sub-stepped internally) |

---

## 3. ARS physics namespace (`EclssParameterBridge`)

The ARS exposes its full model through validated dotted parameters:

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

**Static vs dynamic.** `ars.bed.*` (geometry, cell count) are read once in
`on_configure`; changing them needs a reconfigure cycle to rebuild the beds.
Everything else (heater, cycle, flow, LTL temp, isotherm what-if, efficiency) is
live-tunable and applied on the next step.

**Validation** — the callback rejects out-of-range values with a clear message
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

---

## 4. Simulation controller (ssos_sim)

Set from the GUI SIM-SPEED chips / PARAMS editor or the CLI:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `time_scale` | 1.0 | sim-time acceleration (read live each step) |
| `ic.atmospheric_co2_ppm` | — | initial cabin ppCO₂ |
| `sim_duration_s` | — | run length |

---

## 5. Tuning at runtime

```bash
# Node operating parameters (live):
ros2 param set /crew_node metabolic_scale 1.5      # heavier activity
ros2 param set /ogs_node stack_current_a 46.9      # OGA max -> ~9.25 kg/day
ros2 param set /wrs_node max_urine_process_kg_day 9.0
ros2 param set /simulation_controller time_scale 60.0

# ARS physics (dynamic, validated):
ros2 param set /ars_node ars.heater.total_power_w 800.0

# Rejected with a reason (out of range):
ros2 param set /ars_node ars.isotherm.co2_13x.t0 1.5
```

Faults use these same live parameters plus `enable_auto_faults` — see
[fault_catalog.md](fault_catalog.md).
