cmake_minimum_required(VERSION 3.8)
project(space_station_bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(space_station_bt REQUIRED)
find_package(space_station_bt_nodes REQUIRED)
find_package(space_station_utils REQUIRED)
find_package(space_station_lifecycle_manager REQUIRED)

add_executable(bt_navigator_node src/bt_navigator.cpp)
ament_target_dependencies(bt_navigator_node
  rclcpp
  rclcpp_lifecycle
  space_station_bt
  space_station_bt_nodes
  space_station_utils
)

add_executable(bt_demo src/demo_executor.cpp)
ament_target_dependencies(bt_demo
  rclcpp
  rclcpp_lifecycle
  space_station_bt
  space_station_bt_nodes
  space_station_lifecycle_manager
  space_station_utils
)

install(TARGETS bt_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bt_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES failsafe_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
