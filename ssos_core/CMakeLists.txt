cmake_minimum_required(VERSION 3.14)
project(ssos_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Include headers
include_directories(include)

# System manager library (for testability)
add_library(system_manager_lib
  src/system_manager.cpp
)
ament_target_dependencies(system_manager_lib
  rclcpp
  rclcpp_lifecycle
  space_station_interfaces
  std_msgs
  lifecycle_msgs
)

# System manager executable
add_executable(system_manager src/system_manager_main.cpp)
target_link_libraries(system_manager system_manager_lib)
ament_target_dependencies(system_manager
  rclcpp
  rclcpp_lifecycle
)

# Install
install(TARGETS system_manager_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS system_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_system_manager test/test_system_manager.cpp)
  target_include_directories(test_system_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_system_manager system_manager_lib)
  ament_target_dependencies(test_system_manager
    rclcpp
    rclcpp_lifecycle
    space_station_interfaces
  )
endif()

ament_package()