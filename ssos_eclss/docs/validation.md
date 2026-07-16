# Validation

Validated against Peters, Cmarik & Knox, "4BCO2 EDU Performance",
ICES-2021-313 (2021).

## How to run

```bash
ros2 run ssos_eclss ars_validation ars_validation.csv 180
ros2 run ssos_eclss parameter_sweep parameter_sweep.csv
ros2 run ssos_eclss breakthrough_curve_gen breakthrough_curve.csv 200
```

`ars_validation` returns exit code 0 when the design-point CO2 removal lies in
the paper band, and writes a CSV time history of removal, scrubbed CO2, system
pressure drop, bed and precooler temperatures and blower flow.

## Targets and status

| Quantity | Paper target | Model | Status |
|----------|-------------|-------|--------|
| CO2 removal @ 2 torr, 26 SCFM | 4.16 - 4.76 kg/day | ~4.26 kg/day | ✅ in band |
| System ΔP | ~37-40 in-H2O (full path) | bed component only, scaled by blower curve | ✅ order of magnitude |
| Net efficiency | 0.82 - 0.84 | 0.84 (configurable) | ✅ |
| Holdup loss | 8 - 12% | 10% (configurable) | ✅ |
| Bed regen temperature | ~400 °F with 700 W | reaches with all-19-element heat | ✅ |

The CO2-removal design point is verified by the integration test
`test_four_bed_system.DesignRemovalMatchesPaper` and by the `ars_validation`
standalone.

## Notes on ARS transient fidelity

The breakthrough onset produced by the cell-resolved bed is earlier than the
paper's ~140-145 min because the model uses a simplified adsorption-heat /
cooling treatment rather than the EDU's detailed thermal management. The
steady-state CO2-removal design point — the primary validation requirement —
matches the paper band and is independent of this transient detail. Isotherm
affinity, LDF rates, bed geometry and the system efficiency are all ROS-tunable
for further calibration without rebuilding the physics.

## OGS — ISS OGA / AOGA (ICES-2023-311)

| Quantity | Paper | Model | Test |
|----------|-------|-------|------|
| Max O2 @ 46.9 A, 28 cells | 9.25 kg/day | ~9.25 kg/day | `MaxRateMatchesAOGA` |
| Nominal cell voltage | ~1.7 V | ~1.7 V | `CellVoltageNearAOGAOperatingPoint` |
| Stoichiometry | H2 = 2·O2 | exact | `FaradayStoichiometry` |

## Sabatier — ISS reactor (ICES-2018-155)

| Quantity | Paper | Model | Test |
|----------|-------|-------|------|
| Heat of reaction | −165.4 kJ/mol | 165.4 kJ/mol | (param) |
| ISS conversion | ~95% | ~95% at 648 K | `ISSOperatingConversionAbout95Percent` |
| Kinetic limit < 375 °C | yes | yes | `KineticLimitedAtLowTemperature` |
| Equilibrium limit at high T | yes (peaks intermediate) | yes | `EquilibriumLimitedAtHighTemperature` |

## WRS — ISS WRM (ICES-2023-097)

| Quantity | Paper | Model | Test |
|----------|-------|-------|------|
| UPA recovery (US) | 85–87% | 0.87 | `UPARecoversConfiguredFraction` |
| Total urine recovery w/ BPA | ~97–98% | ~98% | `BPARaisesTotalRecoveryToAbout98Percent` |
| Nominal load | 9 kg/day (6-crew) | 9 kg/day | (param) |

## Conservation

Mass conservation is mandatory and is enforced by tests:

- `test_eclss_mass_balance.CO2BudgetCloses` — CO2 in = accumulated + removed +
  leaked, to floating-point round-off.
- `test_eclss_mass_balance.TotalGasMassConservedWithoutSources`.
- OGS/Sabatier stoichiometry tests verify exact mole ratios.
