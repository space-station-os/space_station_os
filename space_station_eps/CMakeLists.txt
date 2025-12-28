cmake_minimum_required(VERSION 3.8)
project(space_station_eps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(msg_files
  "msg/BCDUStatus.msg"
  "srv/Load.srv"
 
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)


include_directories(include)

add_executable(battery_manager_node src/battery_health.cpp)
rosidl_target_interfaces(battery_manager_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(battery_manager_node
  rclcpp
  std_srvs
  sensor_msgs
  diagnostic_msgs
 ament_index_cpp
)

target_link_libraries(battery_manager_node yaml-cpp)


add_executable(bcdu_node src/bcdu_device.cpp)
rosidl_target_interfaces(bcdu_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(bcdu_node
  rclcpp 
  std_msgs sensor_msgs diagnostic_msgs std_srvs space_station_interfaces

)

add_executable(mbsu_device src/mbsu_distributor.cpp)
ament_target_dependencies(mbsu_device
  rclcpp 
  std_msgs sensor_msgs diagnostic_msgs std_srvs

)

add_executable(ddcu_device src/ddcu_device.cpp)
rosidl_target_interfaces(ddcu_device ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(ddcu_device
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  rclcpp_action
  space_station_interfaces
)

add_executable(solar_power src/sarj_mock.cpp)
ament_target_dependencies(solar_power
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
)
 
install(TARGETS
  battery_manager_node
  bcdu_node
  mbsu_device
  ddcu_device
  solar_power
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
