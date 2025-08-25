cmake_minimum_required(VERSION 3.8)
project(space_station_bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(space_station_bt REQUIRED)
find_package(space_station_bt_nodes REQUIRED)
find_package(space_station_utils REQUIRED)
find_package(space_station_lifecycle_manager REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES REQUIRED ncurses)

include_directories(include)

add_library(bt_navigator_lib src/bt_navigator.cpp)
ament_target_dependencies(bt_navigator_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  behaviortree_cpp_v3
  space_station_bt
  space_station_bt_nodes
  space_station_utils
)

target_link_libraries(bt_navigator_lib ${NCURSES_LIBRARIES})
target_include_directories(bt_navigator_lib PUBLIC ${NCURSES_INCLUDE_DIRS})
target_include_directories(bt_navigator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_bt/include>
  $<INSTALL_INTERFACE:../space_station_bt/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_utils/include>
  $<INSTALL_INTERFACE:../space_station_utils/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../space_station_lifecycle_manager/include>
  $<INSTALL_INTERFACE:../space_station_lifecycle_manager/include>
)

target_link_libraries(bt_navigator_lib 
  ${CMAKE_CURRENT_BINARY_DIR}/../space_station_bt/libspace_station_bt.a
)

add_executable(bt_navigator_node src/bt_navigator_node.cpp)
ament_target_dependencies(bt_navigator_node
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp_v3
  space_station_bt
  space_station_bt_nodes
  space_station_utils
)

target_link_libraries(bt_navigator_node bt_navigator_lib ${NCURSES_LIBRARIES})
target_include_directories(bt_navigator_node PUBLIC ${NCURSES_INCLUDE_DIRS})
target_include_directories(bt_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_bt/include>
  $<INSTALL_INTERFACE:../space_station_bt/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_utils/include>
  $<INSTALL_INTERFACE:../space_station_utils/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../space_station_lifecycle_manager/include>
  $<INSTALL_INTERFACE:../space_station_lifecycle_manager/include>
)

add_executable(bt_demo src/demo_executor.cpp)
ament_target_dependencies(bt_demo
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp_v3
  space_station_bt
  space_station_bt_nodes
  space_station_lifecycle_manager
  space_station_utils
)

target_link_libraries(bt_demo bt_navigator_lib ${NCURSES_LIBRARIES})
target_include_directories(bt_demo PUBLIC ${NCURSES_INCLUDE_DIRS})
target_include_directories(bt_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_bt/include>
  $<INSTALL_INTERFACE:../space_station_bt/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../space_station_utils/include>
  $<INSTALL_INTERFACE:../space_station_utils/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../space_station_lifecycle_manager/include>
  $<INSTALL_INTERFACE:../space_station_lifecycle_manager/include>
)

target_link_libraries(bt_demo 
  ${CMAKE_CURRENT_BINARY_DIR}/../space_station_lifecycle_manager/libspace_station_lifecycle_manager.so
)

install(TARGETS bt_navigator_lib bt_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bt_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
