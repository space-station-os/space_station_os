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

# ---- Pure physics library (NO ROS) ----
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
  src/ogs/electrolysis_cell_model.cpp
  src/ogs/electrolysis_stack_model.cpp
  src/ogs/gas_separator_model.cpp
  src/ogs/oxygen_generator_system.cpp
  src/wrs/distillation_model.cpp
  src/wrs/multifiltration_model.cpp
  src/wrs/catalytic_reactor_model.cpp
  src/wrs/water_recovery_system.cpp
  src/sabatier/reactor_kinetics.cpp
  src/sabatier/sabatier_system.cpp
  src/cabin/cabin_atmosphere.cpp
  src/cabin/crew_metabolic_model.cpp
  src/cabin/leak_model.cpp
  src/faults/fault_types.cpp
  src/faults/sensor_fault.cpp
  src/faults/actuator_fault.cpp
  src/faults/thermal_fault.cpp
  src/faults/fault_injector.cpp
)
target_include_directories(eclss_physics PUBLIC include)
# NOTE: eclss_physics links NOTHING from ROS

# ---- Standalone validation executables (NO ROS) ----
add_executable(ars_validation standalone/ars_validation.cpp)
target_link_libraries(ars_validation eclss_physics)

add_executable(parameter_sweep standalone/parameter_sweep.cpp)
target_link_libraries(parameter_sweep eclss_physics)

add_executable(breakthrough_curve_gen standalone/breakthrough_curve_gen.cpp)
target_link_libraries(breakthrough_curve_gen eclss_physics)

# ---- Install ----
install(TARGETS eclss_physics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS ars_validation parameter_sweep breakthrough_curve_gen
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

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

  # ---- Cabin tests ----
  ament_add_gtest(test_cabin_atmosphere test/unit/cabin/test_cabin_atmosphere.cpp)
  target_link_libraries(test_cabin_atmosphere eclss_physics)

  ament_add_gtest(test_crew_metabolic test/unit/cabin/test_crew_metabolic.cpp)
  target_link_libraries(test_crew_metabolic eclss_physics)

  # ---- Faults tests ----
  ament_add_gtest(test_fault_injector test/unit/faults/test_fault_injector.cpp)
  target_link_libraries(test_fault_injector eclss_physics)

  # ---- ECLSS mass-balance integration test ----
  ament_add_gtest(test_eclss_mass_balance test/integration/test_eclss_mass_balance.cpp)
  target_link_libraries(test_eclss_mass_balance eclss_physics)

  ament_add_gtest(test_ars_closed_loop test/integration/test_ars_closed_loop.cpp)
  target_link_libraries(test_ars_closed_loop eclss_physics)

  # ---- OGS / WRS tests ----
  ament_add_gtest(test_electrolysis test/unit/ogs/test_electrolysis.cpp)
  target_link_libraries(test_electrolysis eclss_physics)

  ament_add_gtest(test_water_recovery test/unit/wrs/test_water_recovery.cpp)
  target_link_libraries(test_water_recovery eclss_physics)

  ament_add_gtest(test_sabatier test/unit/wrs/test_sabatier.cpp)
  target_link_libraries(test_sabatier eclss_physics)
endif()

ament_package()
