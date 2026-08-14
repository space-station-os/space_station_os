# EPS Runtime Runbook

This runbook documents the current runtime paths for the existing EPS implementation.

This document is part of the preparatory work for applying the SSOS backbone pattern to EPS. The goal here is not to redesign the EPS model, but to clarify how the current implementation can be built, launched, and verified.

Related issue: #225
Parent issue: #224

## Launch hierarchy

EPS should be verified at two levels before it is connected to the SSOS backbone.

### 1. EPS core / isolated launch

Launch file:

```bash
ros2 launch space_station_eps eps_core.launch.py
```

This mode starts only EPS-owned internal nodes:

* `battery_manager_node`
* `bcdu_node`
* `mbsu_device`
* `ddcu_device`

This mode is intended for minimal subsystem-level smoke testing.

It does not start SARJ, solar array, ECLSS, Thermal, `system_manager`, or `ssos_sim`.

In this mode, BCDU does not receive solar/SARJ boundary input because `/solar_controller/ssu_voltage_v` is not published. This is acceptable for checking that EPS internal nodes start, but it is not sufficient to verify the solar-to-BCDU-to-MBSU-to-DDCU power flow.

### 2. EPS with solar/SARJ boundary input

Launch file:

```bash
ros2 launch space_station_eps eps_with_solar.launch.py
```

This mode starts the EPS internal nodes plus the existing solar/SARJ mock:

* `solar_power`
* `battery_manager_node`
* `bcdu_node`
* `mbsu_device`
* `ddcu_device`

This mode is intended to verify the existing boundary-input flow:

```text
solar/SARJ mock -> BCDU -> MBSU -> DDCU
```

The solar/SARJ mock publishes `/solar_controller/ssu_voltage_v`, which is used by BCDU to determine charge/discharge behavior.

## Build

From the workspace root:

```bash
cd ~/ssos_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro=jazzy
colcon build --packages-up-to space_station_eps --event-handlers console_direct+
source install/setup.bash
```

Observed on `v0.8.9-dev`:

* `space_station_interfaces` built successfully.
* `space_station_eps` built successfully.
* The following EPS executables were built and installed:

  * `battery_manager_node`
  * `bcdu_node`
  * `mbsu_device`
  * `ddcu_device`
  * `solar_power`

Known build note:

* `space_station_interfaces` may emit an `actionlib_msgs` deprecation warning.
* This is not an EPS build failure and is outside the scope of this runbook.

## Run EPS core

Terminal 1:

```bash
cd ~/ssos_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch space_station_eps eps_core.launch.py
```

Expected behavior:

* `battery_manager_node` starts.
* `bcdu_node` starts.
* `ddcu_device` starts.
* `mbsu_device` starts.
* Battery health topics are published for 24 ORUs.
* BCDU, MBSU, DDCU, and EPS diagnostics topics are visible.

Observed startup:

```text
[battery_manager_node] Initializing BatteryManager with 24 ORUs
[mbsu_node] MBSU Node initialized with 12 channels
[ddcu_node] DDCU (DDCU-I) Node initialized
[bcdu_node] BCDU Device READY..
```

Expected known behavior in core mode:

```text
[mbsu_node]: Insufficient healthy channels (found 0)
```

This is expected in `eps_core.launch.py` because the solar/SARJ boundary input is intentionally not started. Therefore, EPS core launch is suitable for node startup and topic smoke testing, but not for end-to-end power-flow verification.

Telemetry check:

```bash
cd ~/ssos_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list | grep -E "battery|bcdu|mbsu|ddcu|eps"
```

Observed topics include:

```text
/battery/battery_bms_0/health
/battery/battery_bms_1/health
...
/battery/battery_bms_23/health
/bcdu/status
/ddcu/input_voltage
/ddcu/output_voltage
/ddcu/temperature
/eps/diagnostics
/mbsu/channel_0/voltage
/mbsu/channel_1/voltage
...
/mbsu/channel_11/voltage
```

## Run EPS with solar/SARJ boundary input

Terminal 1:

```bash
cd ~/ssos_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch space_station_eps eps_with_solar.launch.py
```

Expected behavior:

* `solar_power` starts as `mock_solar_controller`.
* `/solar_controller/ssu_voltage_v` is published.
* `/solar_controller/ssu_power_w` is published.
* `/solar_controller/ssu_current_a` is published.
* `/solar_controller/panel_temperature` is published.
* `/solar_controller/sun_beta_deg` is published.
* BCDU receives solar-controller input.
* MBSU receives channel voltage from BCDU.
* DDCU receives `/ddcu/input_voltage`.
* DDCU publishes `/ddcu/output_voltage`.

Observed startup:

```text
[solar_power] MockSolarController: orbit=5400s, eclipse=2100s, Pmax=84.0 kW, SSU=~160 V
[battery_manager_node] Initializing BatteryManager with 24 ORUs
[mbsu_node] MBSU Node initialized with 12 channels
[ddcu_node] DDCU (DDCU-I) Node initialized
[bcdu_node] BCDU Device READY..
```

Observed power-flow behavior:

```text
[bcdu_node] [BCDU] Discharging batteries to MBSU...
[mbsu_node] Publishing combined voltage 150.00 V from channels 4 and 0
[ddcu_node] [COOLING] Coolant action server not available.
```

The `COOLING` warning is expected in this standalone EPS verification because the Thermal/coolant action server is outside the scope of this launch mode.

Telemetry check:

```bash
cd ~/ssos_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list | grep -E "solar_controller|battery|bcdu|mbsu|ddcu|eps"
```

Observed topics include all EPS core topics plus:

```text
/solar_controller/panel_temperature
/solar_controller/ssu_current_a
/solar_controller/ssu_power_w
/solar_controller/ssu_voltage_v
/solar_controller/sun_beta_deg
```

Example observed telemetry:

```bash
ros2 topic echo /solar_controller/ssu_voltage_v --once
```

Observed:

```text
data: 0.0
```

This can occur during an eclipse-like zero-voltage period. In that case, BCDU can enter discharge mode and maintain the EPS bus through battery discharge.

```bash
ros2 topic echo /bcdu/status --once
```

Observed:

```text
mode: discharge
bus_voltage: 150.0
regulation_voltage: 150.0
current_draw: 127.0
fault: false
fault_message: ''
```

```bash
ros2 topic echo /ddcu/input_voltage --once
```

Observed:

```text
data: 150.0
```

```bash
ros2 topic echo /ddcu/output_voltage --once
```

Observed:

```text
data: 125.3
```

```bash
ros2 topic echo /eps/diagnostics --once
```

Observed:

```text
level: "\0"
name: SolarArrayGimbal
message: Gimbal nominal
hardware_id: ''
values: []
```

## DDCU service check

Service list:

```bash
ros2 service list | grep ddcu
```

Observed:

```text
/ddcu/load_request
/ddcu_node/describe_parameters
/ddcu_node/get_parameter_types
/ddcu_node/get_parameters
/ddcu_node/get_type_description
/ddcu_node/list_parameters
/ddcu_node/set_parameters
/ddcu_node/set_parameters_atomically
```

Service call:

```bash
ros2 service call /ddcu/load_request space_station_interfaces/srv/Load "{load_voltage: 124.5}"
```

Observed response:

```text
response:
space_station_interfaces.srv.Load_Response(success=True, message='Voltage supply successful at 123.877500 V.')
```

Observed DDCU log:

```text
[ddcu_node] [DDCU] Voltage request received: 124.50 V
[ddcu_node] [DDCU] Voltage supplied: 123.88 V (requested 124.50 V)
```

This confirms that the DDCU load service is available and can provide a regulated load voltage when the EPS bus is healthy.

## Known notes

* `eps_core.launch.py` intentionally does not start the solar/SARJ mock.
* In `eps_core.launch.py`, MBSU may repeatedly report `Insufficient healthy channels (found 0)`.
* This is acceptable for EPS node startup verification, but not for end-to-end power-flow verification.
* `eps_with_solar.launch.py` includes the solar/SARJ mock as a boundary input provider.
* The solar/SARJ mock is treated as an environmental/boundary component for EPS verification, not as part of the minimal EPS backbone interface.
* DDCU may emit `[COOLING] Coolant action server not available.` when the Thermal/coolant action server is not running.
* This warning is acceptable for standalone EPS launch verification unless it prevents EPS telemetry from flowing.
* Full SSOS integrated launch orchestration is out of scope for this runbook.
* The future EPS SSOS backbone wrapper should not directly depend on the detailed solar/SARJ mock. It should expose SSOS-level heartbeat, state, and fault behavior through the common backbone interfaces.

## Future follow-up

* Add EPS SSOS backbone wrapper.
* Connect EPS wrapper to `ssos_core/system_manager`.
* Connect simplified EPS state to `ssos_sim` world inputs.
* Add SSOS integrated launch orchestration for selecting subsystems and boundary mocks.
* Consider moving or renaming the solar/SARJ mock later if the subsystem boundary is formalized.

