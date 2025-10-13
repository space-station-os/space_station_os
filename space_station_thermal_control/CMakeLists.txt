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
find_package(space_station_eclss REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



set(msg_files

  "msg/TankStatus.msg"


  "msg/ExternalLoopStatus.msg"
  "srv/VentHeat.srv"
  "msg/ThermalLinkFlows.msg"
  "msg/ThermalLinkFlowsArray.msg"
  "msg/ThermalNodeData.msg"
  "msg/ThermalNodeDataArray.msg"
  "srv/GetSubTopic.srv"
 
  "msg/SolarPanelsQ.msg"

  "action/Coolant.action"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs space_station_eclss
)

ament_export_dependencies(rosidl_default_runtime)



add_executable(radiator src/radiators.cpp)
ament_target_dependencies(radiator rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(radiator ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(thermal_nodes src/thermals_solver.cpp)
ament_target_dependencies(thermal_nodes rosidl_default_generators rclcpp rclcpp_action ament_index_cpp sensor_msgs space_station_eclss diagnostic_msgs yaml-cpp )

add_executable(cooling_server src/cooling.cpp)
ament_target_dependencies(cooling_server rosidl_default_generators  behaviortree_cpp_v3 rclcpp rclcpp_action std_srvs sensor_msgs space_station_eclss diagnostic_msgs)
rosidl_target_interfaces(cooling_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(thermal_nodes ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(thermal_nodes
  yaml-cpp
)

add_executable(demand src/on_demand_publisher.cpp)
ament_target_dependencies(demand rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(demand ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sun_vector src/sun_vector.cpp)
ament_target_dependencies(sun_vector rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(sun_vector ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sun_vector ${BULLET_LIBRARIES})

add_executable(array_absorptivity src/solar_heat_node.cpp)
ament_target_dependencies(array_absorptivity rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(array_absorptivity ${PROJECT_NAME} "rosidl_typesupport_cpp")
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