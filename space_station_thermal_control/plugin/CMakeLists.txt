cmake_minimum_required(VERSION 3.10)
project(thermal_control)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
#find_package(thermal_control REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

# Proto files for custom messages
set(MSGS_PROTOS
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/thermal_controller/ThermalNodeData.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/thermal_controller/ThermalLinkFlow.proto
)

gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "thermal_controller"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${MSGS_PROTOS}
  DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

# Build the plugin as a shared library
add_library(ThermalPlugin SHARED
  thermal_plugin.cc
)

target_include_directories(ThermalPlugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/thermal_control-msgs_genmsg
  ${Protobuf_INCLUDE_DIRS}
)

add_dependencies(ThermalPlugin thermal_control-msgs)
# Link against Gazebo and generated message targets
target_link_libraries(ThermalPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  thermal_control-msgs
  
)


ament_target_dependencies(ThermalPlugin
  rclcpp
  sensor_msgs
  rosidl_default_runtime
  std_srvs
  #thermal_control 
)

# Ensure ROS2 message interfaces are linked
rosidl_target_interfaces(ThermalPlugin
  space_station_thermal_control "rosidl_typesupport_cpp"
)

set_target_properties(ThermalPlugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../lib
)

# Installation instructions
install(TARGETS ThermalPlugin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)