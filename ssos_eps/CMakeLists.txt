cmake_minimum_required(VERSION 3.14)
project(ssos_eps)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ---- Pure physics library (NO ROS) ----
add_library(eps_physics
  src/common/interpolation.cpp
  src/solar/solar_parameters.cpp
  src/solar/solar_array_model.cpp
  src/solar/ssu_model.cpp
  src/battery/battery_parameters.cpp
  src/battery/battery_oru_model.cpp
  src/bcdu/bcdu_parameters.cpp
  src/bcdu/bcdu_model.cpp
  src/system/eps_parameters.cpp
  src/system/eps_system.cpp
)
target_include_directories(eps_physics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# NOTE: eps_physics links NOTHING from ROS

# ---- ROS node library ----
add_library(eps_ros
  src/nodes/eps_parameter_bridge.cpp
  src/nodes/eps_diagnostics.cpp
  src/nodes/eps_node.cpp
)
target_link_libraries(eps_ros eps_physics)
ament_target_dependencies(eps_ros
  rclcpp rclcpp_lifecycle space_station_interfaces
  std_msgs sensor_msgs diagnostic_msgs lifecycle_msgs
)

# ---- Node executable ----
add_executable(eps_node src/main/eps_main.cpp)
target_link_libraries(eps_node eps_ros)
ament_target_dependencies(eps_node rclcpp rclcpp_lifecycle)

# ---- Standalone validation executable (NO ROS) ----
add_executable(eps_orbit_validation standalone/eps_orbit_validation.cpp)
target_link_libraries(eps_orbit_validation eps_physics)

# ---- Install ----
install(TARGETS eps_physics eps_ros
  EXPORT export_ssos_eps
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS eps_node eps_orbit_validation
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ---- Unit tests ----
  ament_add_gtest(test_solar_array test/unit/solar/test_solar_array.cpp)
  target_link_libraries(test_solar_array eps_physics)

  ament_add_gtest(test_ssu test/unit/solar/test_ssu.cpp)
  target_link_libraries(test_ssu eps_physics)

  ament_add_gtest(test_battery_oru test/unit/battery/test_battery_oru.cpp)
  target_link_libraries(test_battery_oru eps_physics)

  ament_add_gtest(test_bcdu test/unit/bcdu/test_bcdu.cpp)
  target_link_libraries(test_bcdu eps_physics)

  # ---- Integration tests ----
  ament_add_gtest(test_eps_orbit test/integration/test_eps_orbit.cpp)
  target_link_libraries(test_eps_orbit eps_physics)

  # ---- ROS lifecycle / parameter tests ----
  # Own DDS domains so /clock and /sim/world_state from other packages' tests
  # running in parallel cannot reach these nodes.
  ament_add_gtest(test_eps_node test/ros/test_eps_node.cpp
    ENV ROS_DOMAIN_ID=61)
  target_link_libraries(test_eps_node eps_ros)
  ament_target_dependencies(test_eps_node rclcpp rclcpp_lifecycle
    space_station_interfaces std_msgs sensor_msgs diagnostic_msgs lifecycle_msgs)

  ament_add_gtest(test_eps_parameter_bridge test/ros/test_eps_parameter_bridge.cpp
    ENV ROS_DOMAIN_ID=62)
  target_link_libraries(test_eps_parameter_bridge eps_ros)
  ament_target_dependencies(test_eps_parameter_bridge rclcpp rclcpp_lifecycle
    space_station_interfaces std_msgs sensor_msgs diagnostic_msgs lifecycle_msgs)
endif()

ament_export_targets(export_ssos_eps HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle space_station_interfaces
  std_msgs sensor_msgs diagnostic_msgs lifecycle_msgs)

ament_package()
