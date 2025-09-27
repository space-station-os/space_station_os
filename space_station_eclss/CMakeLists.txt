cmake_minimum_required(VERSION 3.8)
project(space_station_eclss)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "msg/AstronautHealth.msg"
  "action/AirRevitalisation.action"
  "action/OxygenGeneration.action"
  "action/WaterRecovery.action"
  "src/wrs_systems/srv/RequestProductWater.srv"
  "src/ars_systems/srv/Co2Request.srv"
  "src/ors_systems/srv/O2Request.srv"
  "src/ors_systems/srv/GreyWater.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs
)


install(PROGRAMS
  src/crew_simulation.py
  src/diagnostic_signal.py
  DESTINATION lib/${PROJECT_NAME}
)


include_directories(include)


add_library(ars_systems
  src/ars_systems/ars_systems.cpp
)

ament_target_dependencies(ars_systems
  rclcpp
  rclcpp_action
  std_msgs
  diagnostic_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  std_srvs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ars_systems ${cpp_typesupport_target})


add_library(ogs_systems
    src/ors_systems/src/ogs_systems.cpp
)

ament_target_dependencies(ogs_systems
  rclcpp
  rclcpp_action
  std_msgs
  diagnostic_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  std_srvs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ogs_systems ${cpp_typesupport_target})


add_library(wrs_systems
    src/wrs_systems/src/wrs_systems.cpp
)

ament_target_dependencies(wrs_systems
  rclcpp
  rclcpp_action
  std_msgs
  diagnostic_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  std_srvs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wrs_systems ${cpp_typesupport_target})


# ------------------------------------------------------------------
# executable
# ------------------------------------------------------------------
add_executable(ars src/ars_systems/ars_main_node.cpp)
target_link_libraries(ars ars_systems)
ament_target_dependencies(ars rclcpp)

add_executable(ogs src/ors_systems/src/ogs_main_node.cpp)
target_link_libraries(ogs ogs_systems)
ament_target_dependencies(ogs rclcpp)

add_executable(wrs src/wrs_systems/src/wrs_main_node.cpp)
target_link_libraries(wrs wrs_systems)
ament_target_dependencies(wrs rclcpp)



install(TARGETS
  ars
  ogs
  wrs
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(DIRECTORY behaviortrees/
  DESTINATION share/${PROJECT_NAME}/behaviortrees
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ars_test test/test_ars_action_server.cpp)
  target_include_directories(ars_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ars_test ars_systems)



  ament_add_gtest(ogs_test test/test_ogs_action_server.cpp)
  target_include_directories(ogs_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ogs_test ogs_systems)
endif()

ament_package()
