# Space Station BT Navigator

The `space_station_bt_navigator` package hosts the ROS 2 component that runs behavior trees for the
space station. It mirrors the design of the `nav2_bt_navigator` package from the
Navigation2 stack and works together with the `space_station_bt` core library
and the `space_station_bt_nodes` plugin library.

## Architecture

- **BtNavigator**: a composable node that owns a `BehaviorTreeEngine` instance.
  At startup the navigator loads node plugins (actions, conditions, decorators
  and controls) from the configured libraries. The navigator can then create and
  execute behavior trees expressed in XML.
- **BehaviorTreeEngine**: wraps the BehaviorTree.CPP factory and execution
  utilities. It registers plugins, builds trees from XML text and provides a
  `run()` method that ticks the tree until completion.
- **space_station_bt_nodes**: contains the node plugins that implement system
  behaviors. Plugins can interact with other subsystems (e.g. GNC, thermal,
  communication) through ROS 2 interfaces.

## Integration with Space Station OS

Behavior trees orchestrate tasks across the various subsystems. Actions can call
services or publish commands to existing packages such as
`space_station_gnc` or `space_station_service`. Conditions may subscribe to
status topics to decide how the tree proceeds. By running within a ROS 2
component container, the navigator can be composed alongside the other nodes of
the system, enabling coordinated mission execution.

## Running the Navigator

After building the workspace with `colcon build`, the navigator component can be
launched like any other ROS 2 node:

```bash
ros2 run space_station_bt_navigator bt_navigator
```

The behavior tree XML can be supplied via a parameter or constructed at runtime.
As the tree ticks, node plugins perform the actual spacecraft operations.

### Failsafe Example

The navigator ships with a minimal tree that demonstrates how a failsafe can
trigger CMG unloading.  The tree structure is:

```
Fallback
 ├─ Sequence
 │   ├─ FailsafeCondition (checks `failsafe` Blackboard entry)
 │   └─ ExampleAction
 └─ UnloadCMG (publishes an unload request)
```

Set the `failsafe` parameter to `true` when launching the navigator to see the
`UnloadCMG` action fire and the `torque_controller` zero its output.
The XML for this tree can be found in `failsafe_tree.xml`.

### Running the full example

1. Launch the GNC torque controller which listens for unload requests:

   ```bash
   ros2 run space_station_gnc torque_controller
   ```

2. In another terminal, run the demo executor with the failsafe flag enabled:

   ```bash
   ros2 run space_station_bt_navigator bt_demo --ros-args -p failsafe:=true
   ```

The navigator will execute the behavior tree, the `FailsafeCondition` will fail
and the `UnloadCMG` action publishes a command. The torque controller receives
this message and temporarily zeroes its CMG torque output.