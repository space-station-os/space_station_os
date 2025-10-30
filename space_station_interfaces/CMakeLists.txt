cmake_minimum_required(VERSION 3.8)
project(space_station_interfaces)

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
  # ECLSS
  "eclss/srv/RequestProductWater.srv"
  "eclss/srv/Co2Request.srv"
  "eclss/srv/O2Request.srv"
  "eclss/srv/GreyWater.srv"
  # EPS
  "eps/srv/Load.srv"
  "eps/msg/BCDUStatus.msg"
  # ATCS
  "atcs/msg/ExternalLoopStatus.msg"
  "atcs/msg/SolarPanelsQ.msg"
  "atcs/msg/TankStatus.msg"
  "atcs/msg/ThermalLinkFlows.msg"
  "atcs/msg/ThermalLinkFlowsArray.msg"
  "atcs/msg/ThermalNodeData.msg"
  "atcs/msg/ThermalNodeDataArray.msg"
  "atcs/srv/GetSubTopic.srv"
  "atcs/srv/VentHeat.srv"
  "atcs/action/Coolant.action"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)


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
