cmake_minimum_required(VERSION 3.14)
project(ssos_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Simulation library (exported so other packages' tests can run it)
# Fault description format shared with the subsystems (no ROS).
add_library(ssos_fault_format src/fault_format.cpp)
target_include_directories(ssos_fault_format PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(ssos_sim_lib
  src/simulation_controller.cpp
  src/scenario_loader.cpp
  src/world_model.cpp
)
target_include_directories(ssos_sim_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ssos_sim_lib ssos_fault_format yaml-cpp)
ament_target_dependencies(ssos_sim_lib
  rclcpp
  rclcpp_lifecycle
  space_station_interfaces
  rosgraph_msgs
  std_msgs
  ament_index_cpp
)

# Simulation controller executable
add_executable(simulation_controller src/main.cpp)
target_link_libraries(simulation_controller ssos_sim_lib)
ament_target_dependencies(simulation_controller
  rclcpp
  rclcpp_lifecycle
)

# Install
install(TARGETS ssos_fault_format ssos_sim_lib
  EXPORT export_ssos_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS simulation_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


install(DIRECTORY scenarios/
  DESTINATION share/${PROJECT_NAME}/scenarios
)

install(DIRECTORY include/
  DESTINATION include
)
# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The scenario-by-name test looks up this package's installed share
  # directory; add the install prefix so it resolves without a sourced
  # workspace (e.g. a plain `colcon test`).
  # Own DDS domain: these tests publish /clock and EPS fault events that
  # other packages' tests running in parallel would otherwise receive.
  ament_add_gtest(test_simulation_controller tests/test_simulation_controller.cpp
    ENV ROS_DOMAIN_ID=65
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  target_include_directories(test_simulation_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_simulation_controller ssos_sim_lib)
  ament_target_dependencies(test_simulation_controller
    rclcpp
    rclcpp_lifecycle
    space_station_interfaces
    rosgraph_msgs
  )
  target_compile_definitions(test_simulation_controller PRIVATE
    SSOS_SIM_TEST_SCENARIOS="${CMAKE_CURRENT_SOURCE_DIR}/tests/scenarios")

  ament_add_gtest(test_scenario_loader tests/test_scenario_loader.cpp)
  target_link_libraries(test_scenario_loader ssos_sim_lib)
  target_compile_definitions(test_scenario_loader PRIVATE
    SSOS_SIM_TEST_SCENARIOS="${CMAKE_CURRENT_SOURCE_DIR}/tests/scenarios"
    SSOS_SIM_SCENARIOS="${CMAKE_CURRENT_SOURCE_DIR}/scenarios")
endif()

ament_export_targets(export_ssos_sim HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle space_station_interfaces rosgraph_msgs
  std_msgs ament_index_cpp yaml-cpp)

ament_package()