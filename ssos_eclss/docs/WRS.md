# WRS — Water Recovery System

Detailed implementation reference for the ECLSS **Water Recovery System**: the
process physics and the ROS 2 node that emulates the ISS Water Recovery and
Management (WRM) hardware.

Primary reference: Williamson, Wilson, Robinson & Luong, *"Status of ISS Water
Management and Recovery"*, ICES-2023-097 (2023).

---

## 1. The real ISS subsystem

The ISS WRS turns wastewater into potable water. Its inputs are **crew urine**,
**humidity condensate**, and (when available) **Sabatier product water**; its
output is potable water delivered to the bus (230–280 kPa). It comprises two
assemblies plus a brine processor:

**Urine Processor Assembly (UPA)** — **Vapor Compression Distillation (VCD)**.
Pretreated urine is recirculated through a rotating **Distillation Assembly**
where wastewater is evaporated at low pressure; the vapour is compressed and
condensed to distillate, and brine is concentrated and stored. The UPA was
designed for **9 kg/day** wastewater (6-crew) and recovers **~85–87%** of water
from US-pretreated urine (target 90%; Russian-segment urine 70%).

**Brine Processor Assembly (BPA)** — a technology demonstrator that further
dewaters UPA brine via **membrane distillation with heated forced convection**,
returning water to the condensate stream and raising **total urine water recovery
to ~97–98%**.

**Water Processor Assembly (WPA)** — polishes the combined distillate + condensate:
`External Filter → Mostly Liquid Separator (degas) → 0.5 µm particulate filter →
Multifiltration (MF) bed (adsorbent + ion exchange) → Catalytic Oxidation Reactor
(oxidises VOCs the MF bed misses, kills microbes) → Gas/Liquid Separator → Ion
Exchange Bed (adds iodine biocide)`. Product water quality is tracked by
conductivity.

| Quantity | Value |
|----------|-------|
| UPA recovery (US pretreatment) | 85–87% (target 90%) |
| Nominal wastewater load | 9 kg/day (6-crew) |
| Total urine recovery with BPA | ~97–98% |
| Potable bus pressure | 230–280 kPa |

---

## 2. Code map

Pure physics under `include/ssos_eclss/wrs/` + `src/wrs/`:

| Component | Class / file | Role |
|-----------|--------------|------|
| Parameters | `WrsParameters` (`wrs_parameters.hpp`) | distillation/MF/catalytic + factory defaults |
| UPA + BPA | `DistillationModel` (`distillation_model`) | VCD recovery + brine processing |
| WPA MF | `MultifiltrationModel` (`multifiltration_model`) | conductivity reduction + breakthrough |
| Catalytic | `CatalyticReactorModel` (`catalytic_reactor_model`) | VOC oxidation vs temperature |
| Orchestrator | `WaterRecoverySystem` (`water_recovery_system`) | urine + condensate → potable |

ROS layer: `WrsNode` (`nodes/wrs_node`), `src/main/wrs_main.cpp`.

---

## 3. Physics implementation

### 3.1 UPA + BPA (`DistillationModel`)

```
upa_distillate = min(feed, max_throughput) · upa_recovery        (≈ 0.87)
upa_brine      = feed − upa_distillate
bpa_water      = upa_brine · brine_recovery   (if BPA enabled)    (≈ 0.846)
total          = upa_distillate + bpa_water
overall recovery = total / feed                                  (≈ 0.98)
energy         = total · specific_energy   (≈ 110 Wh/kg)
```

The BPA stage is toggleable; with it disabled the model reproduces UPA-only
~87% recovery, with it enabled the **~98% total urine recovery** headline.

### 3.2 WPA multifiltration (`MultifiltrationModel`)

Removal efficiency degrades linearly as captured contaminant mass approaches the
bed capacity; once exhausted the bed is `broken_through`:

```
product_conductivity = feed_conductivity · (1 − efficiency·(1 − utilisation))
```

### 3.3 Catalytic oxidation (`CatalyticReactorModel`)

Conversion of volatile organics rises with reactor temperature above an
activation temperature toward an asymptotic maximum:

```
X(T) = X_max · (1 − exp(−(T − T_act)/scale)),   T > T_act
```

### 3.4 Orchestration (`WaterRecoverySystem`)

UPA distillate (+ BPA water) is combined with humidity condensate, the volatile
load is oxidised by the catalytic reactor, and the stream is polished by the MF
bed. Outputs: potable water rate, product conductivity, overall recovery, VOC
conversion, a potable-spec flag (≤ 100 µS/cm) and an MF breakthrough flag.

---

## 4. ROS 2 implementation (ISS emulation)

`WrsNode` is a `rclcpp_lifecycle::LifecycleNode` owning one
`WaterRecoverySystem`. Standard lifecycle; registers with the `system_manager`
on activation.

### Interfaces

| Direction | Topic / service | Type |
|-----------|-----------------|------|
| sub | `/ssos/crew/urine_kg_day` | `std_msgs/Float64` — urine (+ flush) feed |
| sub | `/ssos/crew/latent_water_kg_day` | `std_msgs/Float64` — humidity condensate feed |
| sub | `/ssos/crew/potable_demand_kg_day` | `std_msgs/Float64` — crew drinking draw |
| sub | `/ssos/ogs/water_demand_kg_day` | `std_msgs/Float64` — OGS electrolysis draw |
| sub | `/ssos/sabatier/water_kg_day` | `std_msgs/Float64` — Sabatier product water |
| pub | `/ssos/wrs/potable_kg_day` | `std_msgs/Float64` — instantaneous recovery rate |
| pub | `/ssos/wrs/potable_available_kg` | `std_msgs/Float64` — **potable tank inventory** |
| pub | `/ssos/wrs/wastewater_kg` | `std_msgs/Float64` — **wastewater (WSTA) inventory** |
| pub | `/ssos/wrs/diagnostics` | `diagnostic_msgs/DiagnosticArray` (conductivity, recovery, VOC, tanks, `days_of_supply`, `upa_processing`) |
| pub | `/ssos/wrs/heartbeat` | `SubsystemHeartbeat` |
| pub | `/ssos/fault_event` | `FaultEvent` |
| client | `/ssos/register_subsystem` | `RegisterSubsystem` |

### Water inventories & UPA batch cycle

Unlike a pure flow model, the node maintains two inventories so the station-level
water balance is explicit:

- **Wastewater tank (WSTA)** — accumulates crew urine + flush. Capacity
  `wastewater_capacity_kg` (default 22 kg).
- **Potable tank** — filled by recovered water (UPA distillate + condensate +
  Sabatier), drained by crew drinking and OGS feedwater. Capacity
  `potable_capacity_kg` (default 1000 kg, starting `initial_potable_kg` = 300).

The UPA runs as a **batch**, mirroring the ISS WSTA 70 % trigger:

```
if  wastewater ≥ upa_start_fraction · capacity   (0.70 → 15.4 kg)  → start UPA
process at max_urine_process_kg_day (default 13.6) until
    wastewater ≤ upa_stop_fraction · capacity     (0.05 → 1.1 kg)  → stop UPA
```

At the 4-crew urine load (~6 kg/day) this is roughly a ~2.4-day fill then a
~2-day drain per batch in realtime (faster under `time_scale`). Recovered
distillate (~85 % of processed urine) plus condensate and Sabatier water flow
into the potable tank. `days_of_supply` = potable ÷ daily potable demand.

Each tick: accumulate urine, run the UPA batch logic, advance
`WaterRecoverySystem.step(urine_processed, condensate, voc)` for the quality
physics, update both tanks, serve crew + OGS draws, publish rates + inventories.

**Fault detection (opt-in).** Off by default. With `enable_auto_faults:=true`,
product conductivity above `potable_limit_us` (or MF breakthrough) raises a
`water_quality_out_of_spec` `CRITICAL` fault and an unhealthy heartbeat.

### Parameters

```
step_rate_hz               # node step rate [Hz]
potable_limit_us           # potable conductivity limit [uS/cm] (fault threshold)
wastewater_capacity_kg     # WSTA tank capacity [kg] (default 22)
potable_capacity_kg        # potable tank capacity [kg] (default 1000)
initial_potable_kg         # starting potable inventory [kg] (default 300)
max_urine_process_kg_day   # UPA throughput when running (default 13.6)
upa_start_fraction         # WSTA fill fraction that starts a batch (default 0.70)
upa_stop_fraction          # WSTA fraction that ends a batch (default 0.05)
potable_reserve_kg         # reserve below which OGS draw is starved (default 5)
enable_auto_faults         # opt-in threshold faults (default false)
```

Recovery fractions, BPA enable/recovery, MF capacity and catalytic parameters
live in the physics factory defaults (extendable to the bridge).

### How it emulates the ISS unit

The node reproduces the WRM flow and the flight-control "water balance": crew
urine and condensate (plus Sabatier water) fill the wastewater/potable tanks, the
UPA batch-processes on the WSTA trigger through the UPA→BPA→WPA chain at the
paper's recovery fractions and conductivity, and potable water is drawn by the
crew and OGS. Toggling the BPA emulates the station before/after the
brine-processor demonstration (87 % vs 98 % urine recovery).

---

## 5. Running & validating

```bash
ros2 launch ssos_eclss eclss.launch.py           # full loop feeds the WRS
ros2 topic echo /ssos/wrs/potable_available_kg   # watch the potable tank
ros2 topic echo /ssos/wrs/wastewater_kg          # watch the WSTA fill/drain
ros2 param set /wrs_node max_urine_process_kg_day 9.0
```

Tests (`test_water_recovery`) include `UPARecoversConfiguredFraction` (87%),
`BPARaisesTotalRecoveryToAbout98Percent`, `BPADisabledGivesUPAOnly`,
`Multifiltration.*`, `Catalytic.*`, and `WaterRecovery.*`.
