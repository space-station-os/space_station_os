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
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# ---- Pure physics library (NO ROS) ----
add_library(thermal_network_physics
  src/network/thermal_network.cpp
  src/coolant/coolant_loop.cpp
)
target_link_libraries(thermal_network_physics yaml-cpp)
target_include_directories(thermal_network_physics PUBLIC include)
# NOTE: thermal_network_physics links NOTHING from ROS

# ---- ROS node library ----
add_library(thermal_network_ros
  src/nodes/thermal_network_node.cpp
  src/nodes/coolant_node.cpp
  src/nodes/thermal_diagnostics.cpp
)
target_link_libraries(thermal_network_ros thermal_network_physics)
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

add_executable(sun_vector_node src/nodes/sun_vector_node.cpp)
ament_target_dependencies(sun_vector_node rclcpp geometry_msgs)

add_executable(solar_heat_node src/nodes/solar_heat_node.cpp)
ament_target_dependencies(solar_heat_node rclcpp geometry_msgs space_station_interfaces)

# ---- Install ----
install(TARGETS thermal_network_physics thermal_network_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS thermal_network_node coolant_node sun_vector_node solar_heat_node
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

  ament_add_gtest(test_thermal_network test/unit/network/test_thermal_network.cpp)
  target_link_libraries(test_thermal_network thermal_network_physics)

  ament_add_gtest(test_coolant_loop test/unit/coolant/test_coolant_loop.cpp)
  target_link_libraries(test_coolant_loop thermal_network_physics)

  ament_add_gtest(test_thermal_network_node test/ros/test_thermal_network_node.cpp)
  target_link_libraries(test_thermal_network_node thermal_network_ros)
  ament_target_dependencies(test_thermal_network_node rclcpp rclcpp_lifecycle
    space_station_interfaces diagnostic_msgs lifecycle_msgs)

  ament_add_gtest(test_coolant_node test/ros/test_coolant_node.cpp)
  target_link_libraries(test_coolant_node thermal_network_ros)
  ament_target_dependencies(test_coolant_node rclcpp rclcpp_lifecycle rclcpp_action
    space_station_interfaces diagnostic_msgs lifecycle_msgs)
endif()

ament_package()
