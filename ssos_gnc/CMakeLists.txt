cmake_minimum_required(VERSION 3.14)
project(ssos_gnc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(gnc_common
  src/common/frames.cpp
  src/common/cmg_kinematics.cpp
  src/generated/cmg_jacobian.cpp
)
target_include_directories(gnc_common PUBLIC include)
target_link_libraries(gnc_common Eigen3::Eigen)

set_source_files_properties(src/generated/cmg_jacobian.cpp
  PROPERTIES COMPILE_OPTIONS "-w")

add_library(gnc_plant
  src/plant/inertia.cpp
  src/plant/rigid_body.cpp
  src/plant/disturbance_torques.cpp
  src/plant/sensor_error_model.cpp
)
target_include_directories(gnc_plant PUBLIC include)
target_link_libraries(gnc_plant gnc_common)

add_library(gnc_flight
  src/flight/control/control_parameters.cpp
  src/flight/control/attitude_controller.cpp
  src/flight/actuators/cmg_steering.cpp
  src/flight/actuators/thruster_allocation.cpp
  src/flight/modes/actuation_mode_machine.cpp
  src/flight/estimation/attitude_estimator.cpp
  src/flight/faults/fault_types.cpp
  src/flight/faults/fault_injector.cpp
  src/flight/attitude_control_system.cpp
)
target_include_directories(gnc_flight PUBLIC include)
target_link_libraries(gnc_flight gnc_common)

add_library(gnc_ros
  src/nodes/gnc_parameter_bridge.cpp
  src/nodes/gnc_diagnostics.cpp
  src/nodes/thruster_geometry_loader.cpp
  src/nodes/attitude_control_node.cpp
  src/nodes/gnc_plant_node.cpp
)
target_include_directories(gnc_ros PUBLIC include)
target_link_libraries(gnc_ros gnc_common gnc_plant gnc_flight ${YAML_CPP_LIBRARIES})
ament_target_dependencies(gnc_ros
  rclcpp rclcpp_lifecycle lifecycle_msgs space_station_interfaces
  std_msgs geometry_msgs sensor_msgs diagnostic_msgs urdf
)

add_executable(attitude_control_node src/main/attitude_control_main.cpp)
target_link_libraries(attitude_control_node gnc_ros)
ament_target_dependencies(attitude_control_node rclcpp rclcpp_lifecycle)

add_executable(gnc_plant_node src/main/gnc_plant_main.cpp)
target_link_libraries(gnc_plant_node gnc_ros)
ament_target_dependencies(gnc_plant_node rclcpp rclcpp_lifecycle)

add_executable(attitude_validation standalone/attitude_validation.cpp)
target_link_libraries(attitude_validation gnc_common gnc_plant gnc_flight)

add_executable(control_only_smoke standalone/control_only_smoke.cpp)
target_link_libraries(control_only_smoke gnc_common gnc_flight)

add_executable(mode_sweep standalone/mode_sweep.cpp)
target_link_libraries(mode_sweep gnc_common)

install(TARGETS gnc_common gnc_plant gnc_flight gnc_ros
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS attitude_control_node gnc_plant_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS attitude_validation control_only_smoke mode_sweep
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts)

ament_export_include_directories(include)
ament_export_libraries(gnc_common gnc_plant gnc_flight gnc_ros)
ament_export_dependencies(Eigen3 rclcpp rclcpp_lifecycle space_station_interfaces)

ament_package()
