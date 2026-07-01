# ARS — Air Revitalization System (4-Bed Molecular Sieve)

Detailed implementation reference for the ECLSS **Air Revitalization System**:
the physics model, its governing equations, and the ROS 2 node that emulates the
actual ISS CO₂-removal assembly.

Primary references: Peters, Cmarik & Knox, *"4BCO2 EDU Performance"*,
ICES-2021-313 (2021); Knox & Cmarik, *"CO₂ Removal for the ISS — 4-Bed Molecular
Sieve Material Selection and System Design"* (2019).

---

## 1. The real ISS subsystem

The ISS removes metabolic CO₂ with the **Carbon Dioxide Removal Assembly (CDRA)**,
a 4-Bed Molecular Sieve (4BMS). The 4BCO2 EDU (Engineering Development Unit) is
the test article the physics here is calibrated to. Its operating principle:

- Process air is pulled from the cabin by a **blower** (~26 SCFM).
- A **precooler** chills the air against the Low-Temperature Loop (LTL) coolant,
  because CO₂ adsorption capacity collapses if the air is too warm.
- The air passes through a **desiccant bed** (silica gel + 13X) that removes water
  vapour — water would otherwise out-compete CO₂ for adsorption sites.
- The dried air passes through an **adsorbent bed** (Grace 544 zeolite 13X) that
  captures the CO₂.
- There are **two of each bed** arranged as two trains. While one train adsorbs,
  the other regenerates: it is heated and exposed to space vacuum to drive the
  CO₂ off, which is either vented or routed to the **Sabatier** reactor.
- An **air-save pump** recovers cabin air from the void of the regenerating bed
  before it is exposed to vacuum, so that air is not lost overboard.
- The trains swap on a fixed **half-cycle** of **10-60-10 minutes**
  (air-save / desorb / vacuum), 80 minutes total.

Validated performance targets (from the paper):

| Quantity | Value |
|----------|-------|
| CO₂ removal @ 2 torr ppCO₂, 26 SCFM | 4.16–4.76 kg/day |
| System pressure drop | ~37–40 in-H₂O |
| Bed regeneration temperature | ~400 °F with 700 W |
| CO₂ breakthrough onset | ~140–145 min |
| Net capture efficiency | 0.82–0.84 (holdup loss 8–12%) |

---

## 2. Code map

Pure physics (no ROS) under `include/ssos_eclss/ars/` + `src/ars/`:

| Component | Class / file | Role |
|-----------|--------------|------|
| Parameters | `ArsParameters` (`ars_parameters.hpp`) | every tunable constant + factory defaults |
| Isotherm | `TothIsotherm`, `competitive_loading` (`adsorption_isotherm`) | equilibrium loading q*(P,T) |
| Bed PDE solver | `BedModel` (`bed_model`) | 1D finite-volume packed-bed |
| Precooler | `PrecoolerModel` (`precooler_model`) | air-to-LTL ε-NTU heat exchanger |
| Blower | `BlowerModel` (`blower_model`) | fan curve vs system resistance |
| Air-save pump | `AirSavePumpModel` (`air_save_pump_model`) | void air recovery |
| Valve | `ValveModel` (`valve_model`) | selector + re-pressurisation |
| Sequencer | `CycleStateMachine` (`cycle_state_machine`) | 10-60-10 half-cycle |
| Orchestrator | `FourBedSystem` (`four_bed_system`) | owns everything; `step(dt, cabin)` |

ROS layer: `ArsNode` (`nodes/ars_node`), `src/main/ars_main.cpp`.

---

## 3. Physics implementation

### 3.1 Equilibrium — Toth isotherm

```
q*(P,T) = q_m(T)·b(T)·P / [1 + (b(T)·P)^t]^(1/t)
q_m(T)  = q_m0·exp[χ·(1 - T/T_ref)]
b(T)    = b0·exp[(ΔH/(R·T_ref))·(T_ref/T - 1)]
t(T)    = t0 + α·(1 - T_ref/T),   clamped to (0,1]
```

Provided for CO₂ on 13X, H₂O on 13X, and H₂O on silica gel. `competitive_loading`
applies an extended-Toth competition term so water strongly suppresses CO₂
capacity. The CO₂ affinity `b0` is calibrated so equilibrium loading at ISS ppCO₂
(~2 torr) is ~2 mol/kg. Analytic `dq*/dP` and numerical `dq*/dT` are provided.

### 3.2 Packed-bed PDEs (per finite-volume cell)

```
Gas mass:     ε ∂cᵢ/∂t = -∂(v cᵢ)/∂z + ε D_ax ∂²cᵢ/∂z² - ρ_bulk ∂qᵢ/∂t
Solid mass:   ∂qᵢ/∂t   = k_LDF,i (qᵢ* - qᵢ)                        (Linear Driving Force)
Gas energy:   ε ρ_g c_pg ∂T_g/∂t = -ρ_g c_pg v ∂T_g/∂z + h_gs a_v (T_s-T_g)
                                    + k_ax ∂²T_g/∂z² - U_wall a_wall (T_g-T_wall)
Solid energy: (1-ε) ρ_s c_ps ∂T_s/∂t = h_gs a_v (T_g-T_s)
                                        + ρ_bulk Σ(ΔHᵢ ∂qᵢ/∂t) + Q_heater
Momentum:     -dP/dz = Ergun(v, ε, dp, ρ, μ)
```

Numerical scheme:
- **Upwind** advection, **central-difference** axial dispersion on a uniform grid
  (default 50 cells, configurable).
- **CFL-limited explicit substepping** for advection/diffusion.
- The stiff gas–solid + wall energy exchange is integrated **semi-implicitly**
  (gas heat capacity is tiny vs the exchange coefficient), so stability does not
  force prohibitively small steps.
- `h_gs` uses the Wakao–Kaguei correlation `Nu = 2 + 1.1·Re^0.6·Pr^(1/3)`.
- During desorption the void gas is **pinned to the vacuum partial pressure**, so
  desorbed CO₂ is swept out by the pump rather than re-adsorbing.

Bed modes: `ADSORBING`, `AIR_SAVE`, `DESORBING`, `VACUUM`, `IDLE`,
`REPRESSURIZING`.

### 3.3 Supporting components

- **Precooler** — ε-NTU counter-flow; air exit approaches the LTL inlet within a
  few K at high UA.
- **Blower** — fan head `ΔP = shutoff·(N/Nref)² − a₂·Q²` intersected with the
  Ergun system resistance; constant-flow or constant-RPM control.
- **Air-save pump** — exponential void pump-down `P → P_ult` with recovered moles
  returned to the cabin.
- **Valve** — finite stroke slew + first-order re-pressurisation (0 → ~800 torr in
  ~15 s) for bumpless train transfer.
- **Cycle state machine** — sequences `AIR_SAVE → DESORB → VACUUM`, swaps the
  adsorbing/regenerating trains each half-cycle, and schedules heater power
  (central 7 of 19 elements during air-save, all 19 during desorption).

### 3.4 System-level removal

```
net CO₂ removal = clamp(gross bed capture, 0, inlet CO₂) × capture_efficiency
```

`capture_efficiency` (default 0.84) lumps the air-save inefficiency and bed
holdup loss the cell model does not resolve, reproducing the paper's 0.82–0.84.

---

## 4. ROS 2 implementation (ISS emulation)

`ArsNode` is a `rclcpp_lifecycle::LifecycleNode` that owns one `FourBedSystem`.

### Lifecycle

| Transition | Action |
|------------|--------|
| `on_configure` | declare params via `EclssParameterBridge`, build `FourBedSystem`, create pubs/sub/clients |
| `on_activate` | activate publishers, start the step timer, register with `system_manager` |
| `on_deactivate` | stop the timer, deactivate publishers |
| `on_cleanup` | tear everything down |

### Interfaces

| Direction | Topic / service | Type |
|-----------|-----------------|------|
| sub | `/ssos/cabin/co2_ppm` | `std_msgs/Float64` — **live cabin ppCO₂ (closed loop, preferred)** |
| sub | `/sim/world_state` | `WorldState` — cabin ppCO₂/temp/pressure fallback until cabin feedback arrives |
| pub | `/ssos/ars/co2_removal_kg_day` | `std_msgs/Float64` — feeds cabin (sink) and Sabatier |
| pub | `/ssos/ars/bed_states` | `std_msgs/Float64MultiArray` [12] — per-bed loading, solid temp, mode |
| pub | `/ssos/ars/cycle_phase` | `std_msgs/Float64MultiArray` [3] — elapsed, half-cycle, adsorbing train |
| pub | `/ssos/ars/diagnostics` | `diagnostic_msgs/DiagnosticArray` (scrubbed CO₂, ΔP, bed temps, blower flow, train) |
| pub | `/ssos/ars/heartbeat` | `SubsystemHeartbeat` |
| pub | `/ssos/fault_event` | `FaultEvent` |
| client | `/ssos/register_subsystem` | `RegisterSubsystem` |

Each tick: read the live cabin ppCO₂, advance `FourBedSystem.step(dt, cabin)`
(split into ≤5 s CFL sub-steps so the cycle tracks accelerated sim time),
publish the removal rate + bed/cycle/diagnostic telemetry + heartbeat. Because
removal is driven by the actual cabin ppCO₂, the cabin↔ARS pair self-regulates.

**Fault detection (opt-in).** Off by default. With `enable_auto_faults:=true`,
if CO₂ removal drops below `co2_required_kg_day` (default 4.16) the node
publishes an edge-triggered `co2_removal_below_requirement` `CRITICAL` fault and
marks the heartbeat unhealthy. It is off by default because, in the closed loop,
removal settles at the crew's *current* production rate (which equals the
requirement), so an always-on threshold there would be a knife-edge. See the
main README §7 for the fault-injection paths.

### Parameters (mapped by `EclssParameterBridge`, validated)

```
ars.bed.{desiccant,adsorbent}.{length,diameter,voidage,particle_diameter,n_cells}   (static)
ars.isotherm.co2_13x.{q_m0,b0,dH,t0}        ars.ldf.adsorbent.{k_co2,k_h2o}
ars.heater.{total_power_w,central_power_w,max_temp_k}
ars.cycle.{air_save_s,adsorb_s,vacuum_s}
ars.operating.{inlet_flow_scfm,inlet_ppco2_torr,ltl_inlet_temp_k,ltl_flow_gpm,
               cabin_temp_k,cabin_pressure_pa,vacuum_pressure_pa}
ars.efficiency.{capture_efficiency,holdup_loss}
```

Plus node-level parameters: `step_rate_hz` (default 10), `co2_required_kg_day`
(4.16, fault threshold), `enable_auto_faults` (false).

Static parameters (`ars.bed.*`) require a reconfigure cycle; the rest are
live-tunable and validated (Toth `t0 ∈ (0,1]`, voidage `∈ (0,1)`, temperatures
`> 0`, etc.) before reaching the physics.

### How it emulates the ISS unit

The node reproduces the CDRA control loop: it samples the live cabin ppCO₂ (from
`cabin_node`, just as the real assembly samples cabin air), runs the same
10-60-10 cycle with train swapping and heater scheduling, and reports the CO₂
removal rate and bed temperatures as telemetry. Removal is fed back into the
cabin balance and forward to Sabatier, so the cabin↔ARS loop is closed. The
physics object is identical to what could run on flight hardware — only the I/O
boundary (ROS topics vs sensors/effectors) differs.

---

## 5. Running & validating

```bash
ros2 launch ssos_eclss ars_only.launch.py          # ROS node, auto-activated
ros2 topic echo /ssos/ars/co2_removal_kg_day
ros2 param set /ars_node ars.heater.total_power_w 800.0

ros2 run ssos_eclss ars_validation out.csv 180     # standalone: confirms 4.16-4.76 kg/day
ros2 run ssos_eclss breakthrough_curve_gen          # breakthrough curve CSV
```

Tests: `test_isotherm`, `test_bed_model`, `test_precooler`, `test_blower`,
`test_valve`, `test_cycle_state_machine`, `test_four_bed_system`,
`test_ars_node`. The design-point removal is checked by
`FourBedSystem.DesignRemovalMatchesPaper` and the `ars_validation` exit code.
