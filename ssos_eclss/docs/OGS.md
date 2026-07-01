# OGS — Oxygen Generation System (PEM water electrolysis)

Detailed implementation reference for the ECLSS **Oxygen Generation System**:
the electrochemical physics and the ROS 2 node that emulates the ISS Oxygen
Generation Assembly (OGA).

Primary reference: Takada, Hornyak, Garr, Van Keuren, Faulkner & ElSherbini,
*"Status of the Advanced Oxygen Generation Assembly"*, ICES-2023-311 (2023).

---

## 1. The real ISS subsystem

The ISS **Oxygen Generation Assembly (OGA)** electrolyses potable water to supply
crew O₂; the upgrade programme is the **Advanced OGA (AOGA)**. Overall reaction:

```
2 H2O → 2 H2 + O2
```

Key facts from the paper:

- Feed water from the ISS potable bus passes through an **Inlet Deionizing Bed**
  (iodine removal, O₂-bubble coalescing), is electrolysed by a **28-cell stack**,
  and the cathode-side gaseous H₂ is separated from water by a **Rotary Separator
  Accumulator (RSA)**.
- The **Power Supply Module (PSM)** drives the stack with a selectable
  **10–46.9 A** in Process mode (1.0 A in Standby).
- At **46.9 A the OGA generates 9.25 kg O₂/day** (20.4 lb/day) — enough for
  **10.88 crew** at 0.85 kg O₂/day/crew.
- Nominal **cell voltage ~1.7 V** (AOGA endurance test).
- The water **recirculation loop** is held at ~24 psia (165 kPa); a hydrogen
  sensor shuts the unit down if 1% H₂ is detected in the O₂ stream.
- Product **O₂ is vented to the cabin**; product **H₂ goes to the Sabatier**
  reactor or is vented to space.

| Quantity | Value |
|----------|-------|
| Cells | 28 (series) |
| Max O₂ | 9.25 kg/day @ 46.9 A |
| Cell voltage | ~1.7 V |
| Per-crew O₂ | 0.85 kg/day |
| Recirc-loop pressure | ~24 psia (165 kPa) |

---

## 2. Code map

Pure physics under `include/ssos_eclss/ogs/` + `src/ogs/`:

| Component | Class / file | Role |
|-----------|--------------|------|
| Parameters | `OgsParameters` (`ogs_parameters.hpp`) | cell/stack/separator/operating + factory defaults |
| Cell | `ElectrolysisCellModel` (`electrolysis_cell_model`) | Nernst + overpotentials + Faraday |
| Stack | `ElectrolysisStackModel` (`electrolysis_stack_model`) | series cells + lumped thermal |
| Separator | `GasSeparatorModel` (`gas_separator_model`) | phase separation (RSA) |
| Orchestrator | `OxygenGeneratorSystem` (`oxygen_generator_system`) | feed/power in, O₂/H₂ out |

ROS layer: `OgsNode` (`nodes/ogs_node`), `src/main/ogs_main.cpp`.

---

## 3. Physics implementation

### 3.1 Cell voltage

```
E_cell = E_rev + η_act + η_ohmic + η_conc
```

- **Reversible (Nernst):** `E_rev = E0(T) + (RT/nF)·ln(pH2·√pO2 / aH2O)`, with
  `E0(T) = 1.229 − 9×10⁻⁴·(T − 298.15)` V and `n = 2`.
- **Activation (Butler–Volmer / Tafel):** `η_act = (RT/αnF)·asinh(i / 2i0)`.
- **Ohmic:** `η_ohmic = i · R_membrane` (area-specific resistance).
- **Concentration:** `η_conc = −(RT/nF)·ln(1 − i/i_lim)`.

The active area (≈230 cm²), exchange-current density and membrane resistance are
calibrated so a cell at **46.9 A and the recirc-loop pressure produces ~1.7 V**.
Voltage efficiency is referenced to the thermoneutral voltage (1.481 V).

### 3.2 Gas production — Faraday's law

```
per cell:  H2 = η_F · I/(2F),   O2 = η_F · I/(4F)
```

`η_F` is the **Faradaic (current) efficiency**, default **0.983**, which makes the
stack deliver the measured 9.25 kg/day at 46.9 A versus the 9.41 kg/day Faradaic
ideal. Water consumption is exactly `2 mol H₂O per mol O₂`. If feed water is
insufficient the effective current (and production) scales down and a
`feedwater_limited` flag is raised.

### 3.3 Stack thermal model

A lumped capacity is heated by the waste heat above thermoneutral,
`(V_cell − 1.481)·I` per cell, and cooled by `UA·(T − T_coolant)`, so the stack
temperature settles where generation balances coolant rejection.

### 3.4 Separator

The RSA removes entrained water from the O₂ product with a configured efficiency
and a small carryover fraction.

---

## 4. ROS 2 implementation (ISS emulation)

`OgsNode` is a `rclcpp_lifecycle::LifecycleNode` owning one
`OxygenGeneratorSystem`. Lifecycle is the standard configure/activate/deactivate/
cleanup pattern; on activation it registers with the `system_manager`.

### Interfaces

| Direction | Topic / service | Type |
|-----------|-----------------|------|
| sub | `/ssos/wrs/potable_available_kg` | `std_msgs/Float64` — **feedwater available from the WRS potable bus** |
| pub | `/ssos/ogs/o2_kg_day` | `std_msgs/Float64` — feeds the cabin |
| pub | `/ssos/ogs/water_demand_kg_day` | `std_msgs/Float64` — feedwater draw, deducted by the WRS |
| pub | `/ssos/ogs/diagnostics` | `diagnostic_msgs/DiagnosticArray` (O₂, H₂, water demand, `feedwater_limited`, stack V/power/temp) |
| pub | `/ssos/ogs/heartbeat` | `SubsystemHeartbeat` |
| pub | `/ssos/fault_event` | `FaultEvent` |
| client | `/ssos/register_subsystem` | `RegisterSubsystem` |

Each tick draws feedwater from the WRS potable bus (limited to what the tank can
supply — if empty the stack is `feedwater_limited` and O₂ output falls), advances
the stack at the commanded current (split into ≤30 s thermal sub-steps for
accelerated time), and publishes O₂ production, H₂ rate (to Sabatier), water
demand (back to the WRS) and stack V/power/temperature. Electrolysis of ~1 L of
water per crew member per day yields ~0.9 kg O₂ — closing the loop against the
crew's O₂ consumption.

**Fault detection (opt-in).** Off by default. With `enable_auto_faults:=true`,
O₂ production below `o2_required_kg_day` raises an `o2_production_low` `CRITICAL`
fault and an unhealthy heartbeat.

### Parameters

```
step_rate_hz           # node step rate [Hz]
stack_current_a        # PSM current [A], selectable 10-46.9 (default 27 ~ 5.3 kg/day)
o2_required_kg_day     # life-support requirement (fault threshold)
enable_auto_faults     # opt-in threshold faults (default false)
```

The full electrochemical parameter set (cell area, i0, R_membrane,
faradaic_efficiency, n_cells, …) lives in the physics factory defaults and can be
extended to the parameter bridge for live tuning.

### How it emulates the ISS unit

The node mirrors the OGA control concept: a selectable PSM current drives the
electrolysis stack, feedwater is pulled from the potable bus (as the OGA draws
from the WPA), the model returns the resulting O₂ (to cabin) and H₂ (to
Sabatier), and the stack thermal state evolves like the real
recirculation-loop-cooled stack. Setting `stack_current_a` to 46.9 A reproduces
the OGA's maximum 9.25 kg/day; lower currents emulate part-load operation for a
smaller crew. When the WRS potable tank runs dry the stack throttles, exactly as
electrolysis is gated by available water on-station.

---

## 5. Running & validating

```bash
ros2 launch ssos_eclss ogs_only.launch.py
ros2 topic echo /ssos/ogs/o2_kg_day
ros2 param set /ogs_node stack_current_a 46.9      # OGA maximum -> ~9.25 kg/day
```

Tests (`test_electrolysis`) include `MaxRateMatchesAOGA` (9.25 kg/day @ 46.9 A),
`CellVoltageNearAOGAOperatingPoint` (~1.7 V), `FaradayStoichiometry`,
`FeedwaterLimiting`, and `StackHeatsUnderLoad`.
