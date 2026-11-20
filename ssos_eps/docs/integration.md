# Station integration

`ssos_eps` is coupled to the rest of the station through topics only. No
package links another's physics; each node can still run on its own.

```
                     /clock, /sim/world_state, /ssos/fault_event
 simulation_controller ---------------------------------------------+
                                                                    v
 ars / ogs / wrs / sabatier --- /ssos/eps/load_request (Load.srv) --> eps_node
          ^                                                        |   ^
          +----------------- /ssos/eps/loads (LoadStatusArray) ----+   |
                                                                   |   |
          /thermal/heat_loads (HeatLoadArray, source "eps") <------+   |
                         |                                             |
                         v                                             |
                 thermal_network --- /thermal/nodes/state -------------+
                                     (itcs_coldplate temperature)

 every node --- /ssos/<name>/heartbeat, /ssos/fault_event ---> system_manager
```

## ECLSS on EPS power

Each ECLSS unit owns an `EpsPowerClient` (`ssos_eclss`):

| Unit | Load id | Priority | Demand |
|------|---------|----------|--------|
| ARS (4BMS) | `eclss_ars` | 0 | bed heaters + blower |
| OGS | `eclss_ogs` | 1 | electrolysis stack power |
| WRS | `eclss_wrs` | 1 | distillation and processing power |
| Sabatier | `eclss_sabatier` | 2 | reactor trim heaters |

ARS, WRS and Sabatier cannot run on part power, so a shed or partly served
unit does nothing; OGS current scales with the share it gets. The launched
station's `loads_parameters.yaml` holds the units' measured nominal draw
(ARS 760 W, OGS 800 W, WRS 60 W) as placeholders, so the station load does not
jump when they register.

The client requests the load (replacing the EPS baseline placeholder of the
same id), re-requests when the demand moves by more than 50 W or 5 %, every
30 s, and whenever EPS stops listing the load (for example after an EPS
restart). It reads `/ssos/eps/loads` to work out how much of the request is
delivered:

| EPS reports | Supply fraction | Unit behaviour |
|-------------|-----------------|----------------|
| not running, or `eps_coupling: false` | 1 | runs on its own, as before the coupling |
| load served in full | 1 | nominal |
| load shed | 0 | OGS: no current, no O2. ARS, WRS, Sabatier: physics held, zero output |
| part served | served / requested | OGS current scales with it; the others stop as when shed |
| request rejected | 0 | as shed |

While not fully powered the unit's heartbeat is unhealthy with the supply state
as its message (`shed by EPS`, `brown-out (40% power)`, ...), so
`system_manager` reports the station DEGRADED and recovers it once power is
back. The demand reported to EPS stays at the full-function value while shed,
so loads do not flap between shed and restored.

## EPS heat into thermal

With `thermal.enable_coupling` true, `eps_node` publishes every step on
`/thermal/heat_loads`:

| Node (`thermal.*` parameter) | Heat |
|------------------------------|------|
| `itcs_coldplate` (`coldplate_node`) | DDCU distribution loss + BCDU conversion loss + battery ORU heat |
| `base_link` (`cabin_node`) | power delivered to loads |

It also reads `itcs_coldplate` from `/thermal/nodes/state` and uses it as the
DDCU and battery cold-plate temperature (diagnostic `coldplate_source` reads
`thermal`, otherwise `config`). The station network
(`ssos_thermal/config/station_thermal_nodes.yaml`) ties both nodes to a fixed
17 °C ITCS sink, so with the default 30 kW load the cold plate settles a few
degrees above the coolant loop.

## Time

All of these nodes run with `use_sim_time` and step on the `/clock` published
by `simulation_controller`, so a sim speed-up advances power, life support and
thermal together. `system_manager`'s heartbeat timeouts and recovery hold are
on wall time.

## End-to-end test

`test_station_integration` (in this package) runs `system_manager`,
`simulation_controller`, `eps_node`, the crew, cabin and the four ECLSS units,
and `thermal_network` on the station network, in one process. It checks that:

1. the station starts already on orbit: NOMINAL with no DEGRADED on the way;
2. every ECLSS unit registered its load and is served, and OGS makes O2;
3. EPS heat reaches `itcs_coldplate` and `base_link`, and EPS reads the
   cold-plate temperature back from thermal;
4. a drill tripping the DDCU that feeds OGS (through `/sim/inject_fault`) drops
   the OGS load, stops O2 production and makes the station DEGRADED, and a
   second drill is refused while it runs;
5. `clear_faults` restores the DDCU and OGS, and the station goes
   NOMINAL -> DEGRADED -> RECOVERY -> NOMINAL.

## Running the full station

```bash
ros2 launch space_station space_station.launch.py
```

starts core, sim, ECLSS, EPS, thermal (`thermals.launch.py`, station network)
and the mission-control GUI. The ECLSS loads show in the EPS panel's load
list, and the Thermal panel shows the cold-plate node.
