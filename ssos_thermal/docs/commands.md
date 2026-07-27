# Commands

Quick reference for building, running, inspecting, and shutting down
`ssos_thermal`, from the repo root. All commands run inside the pixi/ROS 2
Jazzy env (`pixi run ...` or `pixi shell` first).

## Build

| Phase | Command |
|---|---|
| Build | `pixi run bash -c "colcon build --symlink-install --packages-select ssos_thermal"` |

## Launch

| Phase | Command | Notes |
|---|---|---|
| Launch | `pixi run bash -c "source install/setup.bash && ros2 launch ssos_thermal thermal.launch.py"` | Starts `thermal_network_node` (autostarts to `ACTIVE`), `sun_vector_node`, `solar_heat_node`, `thermal_visualization.py` |

## Inspect

| Phase | Command | Notes |
|---|---|---|
| Inspect | `pixi run bash -c "source install/setup.bash && ros2 topic list"` | |
| Inspect | `pixi run bash -c "source install/setup.bash && timeout 3 ros2 topic echo <topic> --once"` | `timeout 3` since several topics (`/gnc/pose_all`, `/tcs/ammonia_status`, ...) have no publisher without the rest of the station running |
| Inspect | `pixi run bash -c "source install/setup.bash && ros2 lifecycle get /thermal_network"` | Current state, e.g. `active [3]` |
| Inspect | `pixi run bash -c "source install/setup.bash && ros2 lifecycle list /thermal_network"` | Transitions available *from* the current state |
| Inspect | `pixi run bash -c "source install/setup.bash && rqt"` | **Not available** — `rqt` isn't a pixi dependency in this project's env |

## Test

| Phase | Command | Notes |
|---|---|---|
| Test | `pixi run bash -c "colcon test --packages-select ssos_thermal"` | Runs both gtest binaries: `test_thermal_network` (physics-only) and `test_thermal_network_node` (lifecycle) |
| Test | `pixi run bash -c "colcon test-result --verbose"` | Pass/fail summary after a test run |
| Test | `pixi run bash -c "colcon test --packages-select ssos_thermal --event-handlers console_direct+"` | Streams each gtest's `[ RUN ]`/`[ OK ]` output live, not just the summary |
| Test | `pixi run bash -c "python3 -m py_compile ssos_thermal/scripts/thermal_visualization.py"` | Syntax-checks the viewer script (plain Python, not run via colcon/gtest) |
| Test (repo-wide) | `pixi run test` | The project's own pixi task — currently `colcon test --packages-select ssos_eclss && colcon test-result --verbose`; does **not** cover `ssos_thermal` unless that task is updated |

## Shutdown

| Phase | Command | Notes |
|---|---|---|
| Shutdown | `Ctrl+C` in the terminal running `ros2 launch` | Sends `SIGINT`, which cascades to all child processes and lets `thermal_network` run `on_deactivate`/`on_cleanup` before exit |
| Force shutdown | `Ctrl+C` a second time in that same terminal | `ros2 launch`'s own escalation: if a node doesn't exit within its shutdown timeout after the first `SIGINT`, a second one force-kills the whole process tree immediately, skipping `on_deactivate`/`on_cleanup` |
| Force shutdown | `pkill -9 -f "ros2 launch ssos_thermal"` | Use if the launch terminal itself is gone/unresponsive (e.g. killed the shell, not the launch) — kills the `ros2 launch` process outright; orphaned children usually follow, but verify with the next command |
| Force shutdown | `pkill -9 -f "install/ssos_thermal/lib/ssos_thermal/"` | Belt-and-suspenders: force-kills `thermal_network_node`, `sun_vector_node`, `solar_heat_node` directly by install path if any survive the above |

## Verify

| Phase | Command | Notes |
|---|---|---|
| Verify | `ps aux \| grep -E "ros2 launch\|thermal_network_node\|sun_vector_node\|solar_heat_node\|thermal_visualization" \| grep -v grep` | Lists the full process tree while running (`pixi run` wrapper -> `ros2 launch` -> the four node/script processes); **empty output after shutdown confirms everything actually exited** rather than assuming a signal worked |
