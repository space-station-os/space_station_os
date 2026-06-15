# Architecture

`ssos_eclss` is built in three strictly separated layers.

```
+-------------------------------------------------------------+
| Layer 3: ROS (src/nodes, src/main)                          |
|   ArsNode / OgsNode / WrsNode / CabinNode (LifecycleNode)   |
|   EclssParameterBridge   EclssDiagnostics                   |
+----------------------------+--------------------------------+
                             | (owns, drives step())
                             v
+-------------------------------------------------------------+
| Layer 2: subsystem physics (NO ROS)                         |
|   ars/   ogs/   wrs/   sabatier/   cabin/   faults/         |
+----------------------------+--------------------------------+
                             | (uses)
                             v
+-------------------------------------------------------------+
| Layer 1: common physics foundation (NO ROS)                 |
|   units, gas_properties, thermodynamics, fluid_dynamics,    |
|   heat_transfer, numerical/{rk4, finite_volume, cfl}        |
+-------------------------------------------------------------+
```

## The core principle: physics has zero ROS dependencies

`eclss_physics` (Layers 1-2) links nothing from ROS. This is verified by the
standalone executables, which link `eclss_physics` only and build without ROS.
The same physics objects can therefore be embedded in a flight controller or in
this simulation unchanged.

`eclss_ros` (Layer 3) depends on `eclss_physics` plus `rclcpp`,
`rclcpp_lifecycle` and `space_station_interfaces`.

## Data flow (Epic A boundary)

```
ssos_sim  --/sim/world_state-->  ArsNode/CabinNode/...  (cabin conditions in)
ECLSS nodes --/ssos/<name>/heartbeat--> system_manager
ECLSS nodes --/ssos/fault_event-->       system_manager
ECLSS nodes --/ssos/register_subsystem-> system_manager (on activate)
ECLSS nodes --/ssos/<name>/diagnostics--> telemetry consumers
```

Subsystem nodes never reach into the simulator; they only consume
`/sim/world_state`, respecting the Epic A structural-separation boundary.

## Stepping model

Each node owns one top-level physics object (e.g. `ars::FourBedSystem`) and a
wall timer. On every tick it computes `dt`, reads the latest cabin conditions,
calls `object.step(dt, conditions)`, then publishes telemetry, the relevant rate
topic, a heartbeat and any fault events.

## Numerics

The packed-bed PDE uses first-order upwind advection and central-difference
axial dispersion on a uniform finite-volume grid, with CFL-limited explicit
substeps. The stiff gas-solid energy exchange is integrated **semi-implicitly**
so stability does not force prohibitively small steps. Sorption uses a Linear
Driving Force (LDF) rate toward the competitive Toth equilibrium.
