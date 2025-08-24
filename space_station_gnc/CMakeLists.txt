cmake_minimum_required(VERSION 3.8)
project(space_station_gnc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)        # provides target urdf::urdf
find_package(yaml-cpp REQUIRED)    # YAML tables support

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Unloading.action"
  DEPENDENCIES action_msgs std_msgs
)
# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>)

add_executable(space_station_gnc_node src/space_station_gnc.cpp)
add_executable(demo1a_nauka_incident_estimate src/demo1a_nauka_incident_estimate.cpp)
add_executable(demo1b_crisis_mainengine src/demo1b_crisis_mainengine.cpp)
add_executable(demo1c_small_incident src/demo1c_small_incident.cpp)
add_executable(torque_collector src/torque_collector.cpp)
add_executable(physics_motion src/physics_motion.cpp)
add_executable(physics_sensor src/physics_sensor.cpp)
add_executable(orbit_dynamics src/orbit_dynamics.cpp)
add_executable(sense_estimate src/sense_estimate.cpp)
add_executable(control_torque src/control_torque.cpp)

# Core library with YAML/URDF/Eigen dependencies
add_library(thruster_matrix STATIC src/thruster_matrix.cpp)

# --------------------------------------------------------------------------
# Ament dependencies for targets
# --------------------------------------------------------------------------
ament_target_dependencies(space_station_gnc_node rclcpp std_msgs)
ament_target_dependencies(demo1a_nauka_incident_estimate rclcpp std_msgs tf2_geometry_msgs tf2 tf2_ros OpenCV)
ament_target_dependencies(demo1b_crisis_mainengine rclcpp std_msgs tf2_geometry_msgs tf2 tf2_ros OpenCV)
ament_target_dependencies(demo1c_small_incident rclcpp std_msgs tf2_geometry_msgs tf2 tf2_ros OpenCV)
ament_target_dependencies(torque_collector rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(physics_motion rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(orbit_dynamics rclcpp std_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(physics_sensor rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(sense_estimate rclcpp std_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(control_torque rclcpp std_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs visualization_msgs)
ament_target_dependencies(thruster_matrix rclcpp)


# --------------------------------------------------------------------------
# Public include path for the library (exported)
# --------------------------------------------------------------------------
target_include_directories(thruster_matrix
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --------------------------------------------------------------------------
# Linking
# - Link the control nodes with the generated typesupport and thruster_matrix.
# - Link thruster_matrix with Eigen, URDF, and YAML-CPP.
# --------------------------------------------------------------------------
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(control_torque
  ${cpp_typesupport_target}
  thruster_matrix)
target_link_libraries(physics_motion
  ${cpp_typesupport_target}
  thruster_matrix)
target_link_libraries(orbit_dynamics
  ${cpp_typesupport_target}
  thruster_matrix)

target_link_libraries(thruster_matrix
  Eigen3::Eigen
  urdf::urdf
  yaml-cpp
)

ament_export_include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(control_torque ${cpp_typesupport_target})
target_link_libraries(physics_motion ${cpp_typesupport_target})
target_link_libraries(orbit_dynamics ${cpp_typesupport_target})



# Link OpenCV libraries
target_link_libraries(demo1a_nauka_incident_estimate ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})
target_link_libraries(demo1b_crisis_mainengine ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})
target_link_libraries(demo1c_small_incident ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})

# source by c++
install(TARGETS
  space_station_gnc_node
  demo1a_nauka_incident_estimate
  demo1b_crisis_mainengine
  demo1c_small_incident
  torque_collector
  physics_motion
  physics_sensor
  sense_estimate
  control_torque
  thruster_matrix
  DESTINATION lib/${PROJECT_NAME})

# Install public headers so downstream packages can include thruster_matrix.hpp
install(DIRECTORY include/
  DESTINATION include
)

#launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# URDFfile
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# --------------------------------------------------------------------------
# Testing
# - Enable lint auto and add gtest-based unit tests if BUILD_TESTING is on.
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE "flake8;lint_cmake;uncrustify")
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # GoogleTest via ament
  find_package(ament_cmake_gtest REQUIRED)

  # Unit test for thruster_matrix (URDF/YAML/Eigen required)
  ament_add_gtest(test_thruster_matrix
    tests/unit/test_thruster_matrix.cpp
  )
  if(TARGET test_thruster_matrix)
    target_include_directories(test_thruster_matrix PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_thruster_matrix
      thruster_matrix
      Eigen3::Eigen
      yaml-cpp
      urdf::urdf
    )
  endif()
endif()


set(CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config/config.json")
set(OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/jacob_cmgs_casadi.py ${CONFIG_PATH} ${OUTPUT_PATH}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE result
)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "Matrix Code generation failed")
endif()

# --------------------------------------------------------------------------
# Export include dirs for downstream consumers
# --------------------------------------------------------------------------
ament_export_include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()