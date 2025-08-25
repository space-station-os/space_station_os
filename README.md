# Space Station Behavior Tree Core

This package provides the core behavior tree engine for the space station system.

## Package Structure

```
space_station_bt/
├── include/space_station_bt/
│   └── behavior_tree_engine.hpp    # Core behavior tree engine
├── src/
│   └── behavior_tree_engine.cpp    # Engine implementation
└── CMakeLists.txt
```

## Components

### Behavior Tree Engine
- **BehaviorTreeEngine**: Core engine for loading and executing behavior trees
- Plugin management and tree execution
- Status tracking and error handling

## Usage

The engine is used by the behavior tree navigator to execute behavior trees:

```cpp
#include "space_station_bt/behavior_tree_engine.hpp"

space_station_bt::BehaviorTreeEngine engine(plugin_libraries);
auto status = engine.run(tree);
```

## Dependencies

- `behaviortree_cpp_v3`: Core behavior tree library
- `rclcpp`: ROS2 C++ client library
