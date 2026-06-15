# ARS — 4-Bed Molecular Sieve model

Based on Peters, Cmarik & Knox, "4BCO2 EDU Performance", ICES-2021-313 (2021).

## Configuration

Two desiccant beds (silica gel + 13X) and two adsorbent beds (Grace 544 13X)
form two trains. While one train adsorbs, the other regenerates; the trains swap
each half-cycle. Half-cycle timing is **10-60-10** (air-save / desorb / vacuum),
80 minutes total.

## Equilibrium — Toth isotherm

```
q*(P,T) = q_m(T)·b(T)·P / [1 + (b(T)·P)^t]^(1/t)
q_m(T)  = q_m0·exp[χ·(1 - T/T_ref)]
b(T)    = b0·exp[(ΔH/(R·T_ref))·(T_ref/T - 1)]
t(T)    = t0 + α·(1 - T_ref/T),  clamped to (0,1]
```

Parameters are provided for CO2 on 13X, H2O on 13X and H2O on silica gel. Water
strongly suppresses CO2 capacity, captured by an extended (multicomponent) Toth
competition term — which is why the desiccant beds dry the air upstream of the
CO2 beds.

## Bed PDEs (per finite-volume cell)

```
Gas mass:    ε ∂cᵢ/∂t = -∂(v cᵢ)/∂z + ε D_ax ∂²cᵢ/∂z² - ρ_bulk ∂qᵢ/∂t
Solid mass:  ∂qᵢ/∂t   = k_LDF,i (qᵢ* - qᵢ)                       (LDF)
Gas energy:  ε ρ_g c_pg ∂T_g/∂t = -ρ_g c_pg v ∂T_g/∂z
                                   + h_gs a_v (T_s-T_g)
                                   + k_ax ∂²T_g/∂z²
                                   - U_wall a_wall (T_g-T_wall)
Solid energy:(1-ε) ρ_s c_ps ∂T_s/∂t = h_gs a_v (T_g-T_s)
                                       + ρ_bulk Σ(ΔHᵢ ∂qᵢ/∂t) + Q_heater
Momentum:    -dP/dz = Ergun(v, ε, dp, ρ, μ)
```

Time integration is CFL-limited explicit substepping with upwind advection; the
gas-solid and wall energy exchange is semi-implicit for stability. During
desorption the void gas is held at the vacuum partial pressure so desorbed CO2
is swept out rather than re-adsorbing.

The gas-solid heat-transfer coefficient uses the Wakao-Kaguei correlation
`Nu = 2 + 1.1·Re^0.6·Pr^(1/3)`.

## Supporting components

- **Precooler** — air-to-LTL heat exchanger via the ε-NTU method.
- **Blower** — quadratic fan curve intersected with the Ergun system resistance;
  constant-flow or constant-RPM control. Target ~26 SCFM at ~37-40 in-H2O.
- **Air-save pump** — scroll pump recovering cabin air from the void before
  vacuum exposure (exponential pump-down).
- **Selector valve** — finite stroke plus a first-order re-pressurisation model
  (0 → ~800 torr in ~15 s) for bumpless bed transfer.
- **Cycle state machine** — sequences the 10-60-10 half-cycle, swaps trains and
  schedules heater power (central 7 elements during air-save, all 19 during
  desorption).

## System-level efficiency

Net CO2 removal = gross bed capture (bounded by inlet) × `capture_efficiency`,
representing the air-save inefficiency and bed holdup loss (paper: holdup loss
8-12%, net efficiency 0.82-0.84) that the cell-resolved model does not track
individually.

See [validation.md](validation.md) for the comparison to paper data.
