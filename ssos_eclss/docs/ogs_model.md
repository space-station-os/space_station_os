# OGS — PEM water electrolysis model

`2 H2O → 2 H2 + O2`. Oxygen to the cabin, hydrogen to the Sabatier reactor.

## Cell voltage

```
E_cell = E_rev + η_act + η_ohmic + η_conc
```

- **Reversible (Nernst)** — `E_rev = E0(T) + (RT/nF)·ln(pH2·sqrt(pO2)/aH2O)`,
  with `E0(T) = 1.229 - 9e-4·(T-298.15)` V.
- **Activation** — Butler-Volmer / Tafel: `η_act = (RT/αnF)·asinh(i/2i0)`.
- **Ohmic** — `η_ohmic = i·R_membrane` (area-specific resistance).
- **Concentration** — `η_conc = -(RT/nF)·ln(1 - i/i_lim)`.

Voltage efficiency is referenced to the thermoneutral voltage (1.481 V).

## Gas production — Faraday's law

Per cell: `H2 = I/(2F)`, `O2 = I/(4F)`; the stack multiplies by the cell count.
Water consumption is `2 mol H2O per mol O2`. If feed water is insufficient, the
effective current (and thus production) is scaled down and a `feedwater_limited`
flag is raised.

## Stack thermal model

A lumped capacity is heated by the waste heat above thermoneutral
(`(V_cell - V_tn)·I` per cell) and cooled by `UA·(T - T_coolant)`.

## Gas separator

A phase separator removes entrained water from the product gas with a configured
efficiency and a small carryover fraction.

Defaults are OGA-class (28 cells, ~27 A → a few kg O2/day).
