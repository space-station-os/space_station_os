cmake_minimum_required(VERSION 3.8)
project(space_station_control)

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
find_package(actuator_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(msg_files

  "msg/ThrustersCmd.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs actuator_msgs
)

include_directories(include)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(teleop ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(mux src/mux.cpp)
ament_target_dependencies(mux std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(mux ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  teleop
  mux
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install(DIRECTORY config/
#  DESTINATION share/${PROJECT_NAME}/config
# )

# install(PROGRAMS
#  scripts/iss_simulator.py
#  DESTINATION lib/${PROJECT_NAME}
# )
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
