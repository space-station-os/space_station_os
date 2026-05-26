cmake_minimum_required(VERSION 3.8)
project(space_station_thermal_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_executable(radiator src/radiators.cpp)
ament_target_dependencies(radiator rosidl_default_generators rclcpp sensor_msgs space_station_interfaces)
target_link_libraries(radiator ${cpp_typesupport_target})


add_executable(thermal_nodes src/thermals_solver.cpp)
ament_target_dependencies(thermal_nodes rosidl_default_generators rclcpp rclcpp_action ament_index_cpp sensor_msgs space_station_interfaces diagnostic_msgs yaml-cpp )

add_executable(cooling_server src/cooling.cpp)
ament_target_dependencies(cooling_server rosidl_default_generators  behaviortree_cpp_v3 ament_index_cpp rclcpp rclcpp_action std_srvs sensor_msgs space_station_interfaces diagnostic_msgs)
target_link_libraries(cooling_server ${cpp_typesupport_target})

target_link_libraries(thermal_nodes
  yaml-cpp
  ${cpp_typesupport_target}
)

add_executable(demand src/on_demand_publisher.cpp)
ament_target_dependencies(demand rosidl_default_generators rclcpp sensor_msgs space_station_interfaces)
target_link_libraries(demand ${cpp_typesupport_target})

add_executable(sun_vector src/sun_vector.cpp)
ament_target_dependencies(sun_vector rosidl_default_generators rclcpp sensor_msgs space_station_interfaces)
target_link_libraries(sun_vector ${BULLET_LIBRARIES} ${cpp_typesupport_target})

add_executable(array_absorptivity src/solar_heat_node.cpp)
ament_target_dependencies(array_absorptivity rosidl_default_generators rclcpp sensor_msgs space_station_interfaces)
target_link_libraries(array_absorptivity ${cpp_typesupport_target})
target_link_libraries(sun_vector ${BULLET_LIBRARIES})


include_directories(include)


install(TARGETS
  
  cooling_server

  radiator
  thermal_nodes
  demand
  sun_vector
  array_absorptivity
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
   DESTINATION share/${PROJECT_NAME}/config
 )

install(DIRECTORY behaviortrees/
  DESTINATION share/${PROJECT_NAME}/behaviortrees
)

install(PROGRAMS
 scripts/thermal_visualization.py
 DESTINATION lib/${PROJECT_NAME}
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