# WRS — Water Recovery System model

Inputs: urine (wastewater) and humidity condensate. Output: potable water, with
conductivity tracked as the quality metric.

## Urine Processor Assembly — Vapor Compression Distillation

Evaporation, vapour compression and condensation produce distillate and a
concentrated brine reject. Modelled by a configurable recovery fraction
(~0.87), a specific electrical energy (~110 Wh/kg) and a maximum throughput.

```
distillate = min(feed, max_throughput) · recovery_fraction
brine      = feed - distillate
```

## Catalytic reactor

High-temperature catalytic oxidation of trace volatile organics. Conversion
rises with reactor temperature above an activation temperature toward an
asymptotic maximum:

```
X(T) = X_max·(1 - exp(-(T - T_act)/scale)),   T > T_act
```

## Multifiltration (Water Processor Assembly)

Adsorption + ion-exchange beds reduce conductivity. Removal efficiency degrades
linearly as the captured contaminant mass approaches the bed capacity, after
which the bed is `broken_through`:

```
product_conductivity = feed_conductivity · (1 - efficiency·(1 - utilisation))
```

## Orchestration

The UPA distillate is combined with the humidity condensate, polished by the
catalytic reactor and multifiltration, and delivered as potable water. The
result reports product conductivity, overall recovery, VOC conversion, a
potable-spec flag and a multifiltration breakthrough flag.
