cmake_minimum_required(VERSION 3.14)
project(ssos_eclss)

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

include_directories(include)

# ---- Pure physics library (NO ROS) — common + ARS ----
add_library(eclss_physics
  src/common/gas_properties.cpp
  src/common/thermodynamics.cpp
  src/common/fluid_dynamics.cpp
  src/common/heat_transfer.cpp
  src/ars/adsorption_isotherm.cpp
  src/ars/bed_model.cpp
  src/ars/precooler_model.cpp
  src/ars/blower_model.cpp
  src/ars/air_save_pump_model.cpp
  src/ars/valve_model.cpp
  src/ars/cycle_state_machine.cpp
  src/ars/four_bed_system.cpp
)
target_include_directories(eclss_physics PUBLIC include)
# NOTE: eclss_physics links NOTHING from ROS

# ---- ROS node library ----
add_library(eclss_ros
  src/nodes/eclss_parameter_bridge.cpp
  src/nodes/eclss_diagnostics.cpp
  src/nodes/ars_node.cpp
)
target_link_libraries(eclss_ros eclss_physics)
ament_target_dependencies(eclss_ros
  rclcpp rclcpp_lifecycle space_station_interfaces
  std_msgs sensor_msgs diagnostic_msgs lifecycle_msgs
)

# ---- Node executable ----
add_executable(ars_node src/main/ars_main.cpp)
target_link_libraries(ars_node eclss_ros)
ament_target_dependencies(ars_node rclcpp rclcpp_lifecycle)

# ---- Standalone validation executables (NO ROS) ----
add_executable(ars_validation standalone/ars_validation.cpp)
target_link_libraries(ars_validation eclss_physics)

add_executable(parameter_sweep standalone/parameter_sweep.cpp)
target_link_libraries(parameter_sweep eclss_physics)

add_executable(breakthrough_curve_gen standalone/breakthrough_curve_gen.cpp)
target_link_libraries(breakthrough_curve_gen eclss_physics)

# ---- Install ----
install(TARGETS eclss_physics eclss_ros
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS ars_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ars_validation parameter_sweep breakthrough_curve_gen
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gas_properties test/unit/common/test_gas_properties.cpp)
  target_link_libraries(test_gas_properties eclss_physics)

  ament_add_gtest(test_thermodynamics test/unit/common/test_thermodynamics.cpp)
  target_link_libraries(test_thermodynamics eclss_physics)

  ament_add_gtest(test_heat_transfer test/unit/common/test_heat_transfer.cpp)
  target_link_libraries(test_heat_transfer eclss_physics)

  # ---- ARS unit tests ----
  ament_add_gtest(test_isotherm test/unit/ars/test_isotherm.cpp)
  target_link_libraries(test_isotherm eclss_physics)

  ament_add_gtest(test_bed_model test/unit/ars/test_bed_model.cpp)
  target_link_libraries(test_bed_model eclss_physics)

  ament_add_gtest(test_precooler test/unit/ars/test_precooler.cpp)
  target_link_libraries(test_precooler eclss_physics)

  ament_add_gtest(test_blower test/unit/ars/test_blower.cpp)
  target_link_libraries(test_blower eclss_physics)

  ament_add_gtest(test_valve test/unit/ars/test_valve.cpp)
  target_link_libraries(test_valve eclss_physics)

  ament_add_gtest(test_cycle_state_machine test/unit/ars/test_cycle_state_machine.cpp)
  target_link_libraries(test_cycle_state_machine eclss_physics)

  # ---- ARS integration test ----
  ament_add_gtest(test_four_bed_system test/integration/test_four_bed_system.cpp)
  target_link_libraries(test_four_bed_system eclss_physics)

  # ---- ROS lifecycle / parameter tests ----
  ament_add_gtest(test_ars_node test/ros/test_ars_node.cpp)
  target_link_libraries(test_ars_node eclss_ros)
  ament_target_dependencies(test_ars_node rclcpp rclcpp_lifecycle
    space_station_interfaces std_msgs diagnostic_msgs lifecycle_msgs)

  ament_add_gtest(test_parameter_bridge test/ros/test_parameter_bridge.cpp)
  target_link_libraries(test_parameter_bridge eclss_ros)
  ament_target_dependencies(test_parameter_bridge rclcpp rclcpp_lifecycle
    space_station_interfaces std_msgs diagnostic_msgs lifecycle_msgs)
endif()

ament_package()
