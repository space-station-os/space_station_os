cmake_minimum_required(VERSION 3.14)
project(ssos_thermal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ssos_sim REQUIRED)

include_directories(include)

# ---- Pure physics library (NO ROS) ----
add_library(thermal_network_physics
  src/network/thermal_network.cpp
  src/coolant/coolant_loop.cpp
)
target_link_libraries(thermal_network_physics yaml-cpp)
target_include_directories(thermal_network_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# NOTE: thermal_network_physics links NOTHING from ROS

# ---- ROS node library ----
add_library(thermal_network_ros
  src/nodes/thermal_network_node.cpp
  src/nodes/coolant_node.cpp
  src/nodes/thermal_diagnostics.cpp
)
target_link_libraries(thermal_network_ros thermal_network_physics ssos_sim::ssos_fault_format)
ament_target_dependencies(thermal_network_ros
  rclcpp rclcpp_lifecycle rclcpp_action ament_index_cpp
  space_station_interfaces diagnostic_msgs lifecycle_msgs
)

# ---- Node executables ----
add_executable(thermal_network_node src/main/thermal_network_main.cpp)
target_link_libraries(thermal_network_node thermal_network_ros)
ament_target_dependencies(thermal_network_node rclcpp rclcpp_lifecycle)

add_executable(coolant_node src/main/coolant_main.cpp)
target_link_libraries(coolant_node thermal_network_ros)
ament_target_dependencies(coolant_node rclcpp rclcpp_lifecycle rclcpp_action)

# ---- Install ----
install(TARGETS thermal_network_physics thermal_network_ros
  EXPORT export_ssos_thermal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS thermal_network_node coolant_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/thermal_visualization.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)

  ament_add_gtest(test_thermal_network test/unit/network/test_thermal_network.cpp)
  target_link_libraries(test_thermal_network thermal_network_physics)
  target_compile_definitions(test_thermal_network PRIVATE
    SSOS_THERMAL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_coolant_loop test/unit/coolant/test_coolant_loop.cpp)
  target_link_libraries(test_coolant_loop thermal_network_physics)

  ament_add_gtest(test_thermal_diagnostics test/unit/diagnostics/test_thermal_diagnostics.cpp)
  target_link_libraries(test_thermal_diagnostics thermal_network_ros)

  ament_add_gtest(test_thermal_network_node test/ros/test_thermal_network_node.cpp)
  target_link_libraries(test_thermal_network_node thermal_network_ros)
  ament_target_dependencies(test_thermal_network_node rclcpp rclcpp_lifecycle
    space_station_interfaces diagnostic_msgs lifecycle_msgs rosgraph_msgs)
  target_compile_definitions(test_thermal_network_node PRIVATE
    SSOS_THERMAL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(test_coolant_node test/ros/test_coolant_node.cpp)
  target_link_libraries(test_coolant_node thermal_network_ros)
  ament_target_dependencies(test_coolant_node rclcpp rclcpp_lifecycle rclcpp_action
    space_station_interfaces diagnostic_msgs lifecycle_msgs)

  set_tests_properties(test_thermal_network_node PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=214")
  set_tests_properties(test_coolant_node PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=215")
endif()

# Exported so other packages' integration tests can run the thermal nodes.
ament_export_targets(export_ssos_thermal HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle rclcpp_action ament_index_cpp
  space_station_interfaces diagnostic_msgs lifecycle_msgs yaml-cpp ssos_sim)

ament_package()
