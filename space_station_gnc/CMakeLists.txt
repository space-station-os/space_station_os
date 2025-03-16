cmake_minimum_required(VERSION 3.8)
project(space_station_gnc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SDL2 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS})

add_executable(space_station_gnc_node src/space_station_gnc.cpp)
add_executable(demo1a_nauka_incident_estimate src/demo1a_nauka_incident_estimate.cpp)
add_executable(demo1b_crisis_mainengine src/demo1b_crisis_mainengine.cpp)
add_executable(torque_collector src/torque_collector.cpp)
add_executable(physics_motion src/physics_motion.cpp)
ament_target_dependencies(space_station_gnc_node rclcpp std_msgs)
ament_target_dependencies(demo1a_nauka_incident_estimate rclcpp std_msgs tf2_geometry_msgs tf2 tf2_ros OpenCV)
ament_target_dependencies(demo1b_crisis_mainengine rclcpp std_msgs tf2_geometry_msgs tf2 tf2_ros OpenCV)
ament_target_dependencies(torque_collector rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(physics_motion rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

# Link OpenCV libraries
target_link_libraries(demo1a_nauka_incident_estimate ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})
target_link_libraries(demo1b_crisis_mainengine ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})

# source by c++
install(TARGETS
  space_station_gnc_node
  demo1a_nauka_incident_estimate
  demo1b_crisis_mainengine
  torque_collector
  physics_motion
  DESTINATION lib/${PROJECT_NAME})

# source by python
#install(PROGRAMS
#  src/space_station_gnc/demo_crisis.py
#  DESTINATION lib/${PROJECT_NAME})

#launch files
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# URDFfile 
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

#install(DIRECTORY meshes/
#  DESTINATION share/${PROJECT_NAME}/meshes
#)
install(DIRECTORY urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
