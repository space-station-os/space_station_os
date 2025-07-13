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
find_package(std_srvs REQUIRED)
find_package(space_station_eclss REQUIRED)
find_package(Bullet REQUIRED)
find_package(urdf REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${BULLET_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
)


set(msg_files

  "msg/TankStatus.msg"
  "srv/CoolantFlow.srv"
  "srv/InternalLoop.srv"
  "msg/InternalLoopStatus.msg"
  "msg/ExternalLoopStatus.msg"
  "srv/VentHeat.srv"
  "msg/ThermalLinkFlows.msg"
  "msg/ThermalLinkFlowsArray.msg"
  "msg/ThermalNodeData.msg"
  "msg/ThermalNodeDataArray.msg"
  "srv/GetSubTopic.srv"
  "srv/NodeHeatFlow.srv"
  "msg/SolarPanelsQ.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs space_station_eclss
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(coolant src/coolant.cpp)
ament_target_dependencies(coolant rosidl_default_generators rclcpp  std_srvs space_station_eclss)
rosidl_target_interfaces(coolant ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(external_loop src/external_loop.cpp)
ament_target_dependencies(external_loop rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(external_loop ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(radiator src/radiators.cpp)
ament_target_dependencies(radiator rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(radiator ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(thermal_nodes src/thermals_solver.cpp)
ament_target_dependencies(thermal_nodes rosidl_default_generators rclcpp sensor_msgs space_station_eclss)
rosidl_target_interfaces(thermal_nodes ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(thermal_nodes
  ${urdf_LIBRARIES}
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
  coolant
  external_loop
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

# add_subdirectory(plugin)

include_directories(${CMAKE_CURRENT_BINARY_DIR})