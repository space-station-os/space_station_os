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

include_directories(include)

# Simulation library (for testability)
add_library(ssos_sim_lib
  src/simulation_controller.cpp
  src/world_model.cpp
)
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
install(TARGETS ssos_sim_lib
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
# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_simulation_controller tests/test_simulation_controller.cpp)
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
endif()

ament_package()