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


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
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


add_executable(ars src/ars_systems/ars_systems.cpp )
ament_target_dependencies(ars std_srvs rosidl_default_generators rclcpp sensor_msgs rclcpp_action diagnostic_msgs)
rosidl_target_interfaces(ars ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ogs src/ors_systems/src/ogs_systems.cpp)
ament_target_dependencies(ogs std_srvs rosidl_default_generators rclcpp rclcpp_action diagnostic_msgs)
rosidl_target_interfaces(ogs ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wrs src/wrs_systems/src/wrs_systems.cpp)
ament_target_dependencies(wrs rosidl_default_generators rclcpp rclcpp_action diagnostic_msgs std_srvs)
rosidl_target_interfaces(wrs ${PROJECT_NAME} "rosidl_typesupport_cpp")


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
