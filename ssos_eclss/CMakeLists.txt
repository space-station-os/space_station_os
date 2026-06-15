cmake_minimum_required(VERSION 3.14)
project(ssos_eclss)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(space_station_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

# ---- Pure physics library (NO ROS) ----
add_library(eclss_physics
  src/common/gas_properties.cpp
  src/common/thermodynamics.cpp
  src/common/fluid_dynamics.cpp
  src/common/heat_transfer.cpp
)
target_include_directories(eclss_physics PUBLIC include)
# NOTE: eclss_physics links NOTHING from ROS

# ---- Install ----
install(TARGETS eclss_physics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gas_properties test/unit/common/test_gas_properties.cpp)
  target_link_libraries(test_gas_properties eclss_physics)

  ament_add_gtest(test_thermodynamics test/unit/common/test_thermodynamics.cpp)
  target_link_libraries(test_thermodynamics eclss_physics)

  ament_add_gtest(test_heat_transfer test/unit/common/test_heat_transfer.cpp)
  target_link_libraries(test_heat_transfer eclss_physics)
endif()

ament_package()
