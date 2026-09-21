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
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ---- ROS-independent EPS core ----
add_library(eps_core
  src/eps_model.cpp
)

target_include_directories(eps_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- ROS lifecycle wrapper ----
add_library(eps_ros
  src/eps_node.cpp
)

target_link_libraries(eps_ros
  eps_core
)

target_include_directories(eps_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(eps_ros
  rclcpp
  rclcpp_lifecycle
  space_station_interfaces
  diagnostic_msgs
  lifecycle_msgs
)

# ---- Node executable ----
add_executable(eps_node
  src/eps_main.cpp
)

target_link_libraries(eps_node
  eps_ros
)

ament_target_dependencies(eps_node
  rclcpp
  rclcpp_lifecycle
)

# ---- Install ----
install(
  TARGETS eps_core eps_ros
  EXPORT export_ssos_eps
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS eps_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_eps_model
    test/test_eps_model.cpp
  )

  target_link_libraries(test_eps_model
    eps_core
  )

  ament_add_gtest(test_eps_node
    test/test_eps_node.cpp
  )

  target_link_libraries(test_eps_node
    eps_ros
  )

  ament_target_dependencies(test_eps_node
    rclcpp
    rclcpp_lifecycle
    space_station_interfaces
    diagnostic_msgs
    lifecycle_msgs
  )
endif()


ament_export_targets(export_ssos_eps HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
