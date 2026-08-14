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
