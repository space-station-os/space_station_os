# Behavior Tree Architecture

This document details the behavior tree (BT) system implemented for Space Station OS. The design follows the Navigation2 (Nav2) approach and is split across several packages that work together to execute decision logic through behavior tree nodes.

## Packages Overview

1. **space_station_bt** : Contains the core `BehaviorTreeEngine` which wraps the BehaviorTree.CPP library. It registers plugins, creates trees from XML, and ticks the tree.
2. **space_station_bt_nodes** : Holds the node plugin library. It include actions, conditions, controls and decorators, for now mainly :  `FailsafeCondition` and `UnloadCmgAction`.
3. **space_station_bt_navigator** : ROS2 component that holds a `BehaviorTreeEngine` instance. It loads the plugin library at startup and runs specific trees to control the spacecraft subsystems.
4. **space_station_utils** : Collection of helper functions shared across the stack (parameter utilities, plugin loading helpers and more).
5. **space_station_core** : Houses abstract plugin interfaces. Subsystems implement these base classes so that the navigator can load plugins in a generic manner.
6. **space_station_lifecycle_manager** : Brings up and shuts down nodes using the ROS 2 managed lifecycle. It ensures plugins and other nodes are configured and activated before a mission starts.

```
BtNavigator ---> BehaviorTreeEngine ---> Behavior Tree ---> <Plugins | Actions | Conditions | Decorators>
```

## BehaviorTreeEngine

The engine is started with a list of plugin libraries. In construction it registers all available node types with the BehaviorTree.CPP factory. Trees are created from XML strings using the factory and executed by repeatedly ticking the root node until returns success or failure.

Key API functiosn:
- `BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries)` : loads plugins.
- `BT::Tree createTreeFromText(const std::string & xml, BT::Blackboard::Ptr bb)` : builds a tree from the BT XML.
- `Status run(BT::Tree * tree)` : ticks the tree at 10hz until it finishes.

## BtNavigator

`BtNavigator` is a ros2 node. During construction it initializes `BehaviorTreeEngine` and declares all parameters needed by the tree (e.g. `failsafe`). The `execute()` loads the behavior tree XML, creates a blackboard with parameter data, and calls `run()` on the engine.

This can be launched on its own or with other nodes. When the tree is running, individual plugins interact with other subsystems through topics, services or mainly actions.

## Node Plugins

`space_station_bt_nodes` exposes behaviors used in trees. Each plugin registers itself with the factory so the XML parser can start it. Some examples:

- **ExampleAction** : A minimal synchronous action specifically for demonstration and starting point for other SSOS subsystems and developers to build upon.
- **ExampleCondition** : returns success/failure based on defined logic.
- **ExampleDecorator / ExampleControl** : Basic dev how decorators and control nodes are written.
- **FailsafeCondition** : It reads a `failsafe` boolean from its input port. When the value is `true` the condition fails, causing the tree to fall back to recovery behavior.
- **UnloadCmgAction** : publishes a message on `gnc/unload_cmg` to trigger CMG unloading.

Registration occurs in each source file using `BT_REGISTER_NODES`, so the factory can create the node when parsing XML.

## Supporting Packages

Several additional packages mirror the Navigation2 architecture to simplify development and deployment:

- **space_station_utils** provides helper functions such as parameter declaration utilities and plugin loaders used by many nodes.
  The `PluginLoader` wrapper simplifies creating instances from pluginlib while
  logging meaningful errors if loading fails.
- **space_station_core** defines abstract base classes for plugins. Subsystems implement these interfaces so the navigator and other components can load them generically.
- **space_station_lifecycle_manager** implements a node that brings up managed nodes through the ROS&nbsp;2 lifecycle. It configures and activates a list of nodes on startup and can cleanly shut them down.

## Example Workflow

The tree in `space_station_bt_navigator/failsafe_tree.xml` describes how a sequence can fallback to a recovery action. The flow is:

1. `BtNavigator` loads the example XML at startup.
2. `FailsafeCondition` checks the `failsafe` boolean from the blackboard. When `false`, `ExampleAction` runs and the tree succeeds.
3. If `FailsafeCondition` returns failure, control moves to the fallback branch where `UnloadCmgAction` publishes a command.
4. The GNC `torque_controller` node subscribes to `gnc/unload_cmg`. On receiving the message it starts the CMG unloading logic.

## Extending the System

Additional behaviors can be implemented by adding new plugins to `space_station_bt_nodes`. After building the workspace, they become available for use in any XML tree. The navigator can be configured to load different trees or parameterize them through the blackboard.

For further reference, see the code in:
- `space_station_bt/` : core engine implementation
- `space_station_bt_nodes/` : plugin examples
- `space_station_bt_navigator/` : navigator component and example tree
- `space_station_utils/` : miscellaneous helper routines used by multiple nodes
- `space_station_core/` : plugin interface definitions
- `space_station_lifecycle_manager/` : lifecycle management utility node
