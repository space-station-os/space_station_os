# Space Station Behavior Tree Navigator

Behavior tree navigator for space station control operations using ROS 2 and BehaviorTree.CPP.

## Features

- Behavior Tree Engine for executing behavior trees
- Custom BT nodes for space station operations
- Lifecycle management with ROS 2
- Configurable tree loading via parameters

## Building

```bash
colcon build --packages-select space_station_bt_navigator space_station_bt_nodes space_station_bt
```

## Running

```bash
# Direct execution
ros2 run space_station_bt_navigator bt_navigator_node

# Using launch file
ros2 launch space_station_bt_navigator bt_navigator.launch.py

# Demo executor
ros2 run space_station_bt_navigator bt_demo
```

## Parameters

- `tree_xml`: Behavior tree XML file to load
- `failsafe`: Enable/disable failsafe mode
- `plugin_libraries`: BT node plugin libraries to load
- `use_sim_time`: Use simulation time

## Available BT Nodes

### Condition Nodes
- `CheckSystemHealth`, `CheckOrbitStatus`, `CheckPowerStatus`
- `CheckThermalStatus`, `CheckGroundContact`, `CheckAttitude`
- `CheckCMGThreshold`, `CheckCMGUnloadingStatus`

### Action Nodes
- `UnloadCMG`, `SetControlMode`, `LogMessage`
- `WaitForStabilization`, `ExecuteMissionTasks`

## Example Usage

1. **Start the system**:
   ```bash
   ros2 launch space_station_bt_navigator cmg_unloading_test.launch.py
   ```

2. **Test control mode switching**:
   ```bash
   ros2 launch space_station_bt_navigator control_mode_test.launch.py
   ```