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

  "src/ars_systems/msg/AirData.msg"
  "src/ars_systems/srv/CrewQuarters.srv"

  
  "src/ors_systems/msg/Sabatier.msg"
  "src/ors_systems/msg/Water.msg"
  "src/ors_systems/msg/Electrolysis.msg"

  "src/wrs_systems/srv/Water.srv"
  "src/wrs_systems/srv/Upa.srv"
  "src/wrs_systems/srv/Distillation.srv"
  "src/wrs_systems/srv/Filteration.srv"
  "src/wrs_systems/srv/IonBed.srv"
  "src/wrs_systems/srv/CleanWater.srv"
  "src/wrs_systems/msg/WaterCrew.msg"
 
  # CDRA SAFETY
  "src/ars_systems/msg/CdraStatus.msg"

  "action/AirRevitalisation.action"
  "action/OxygenGeneration.action"
  "action/WaterRecovery.action"
  "src/wrs_systems/srv/RequestProductWater.srv"
  "src/ars_systems/srv/CoRequest.srv"
  "src/ors_systems/srv/O2Request.srv"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES  std_msgs sensor_msgs visualization_msgs
)

  



include_directories(include)

add_executable(collector src/ars_systems/air_collector_tank.cpp)
ament_target_dependencies(collector std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(collector ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(desiccant1 src/ars_systems/desiccant_tank_1.cpp)
ament_target_dependencies(desiccant1 std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(desiccant1 ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(desiccant2 src/ars_systems/desiccant_tank_2.cpp)
ament_target_dependencies(desiccant2 std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(desiccant2 ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(adsorbent1 src/ars_systems/adsorbent_tank_1.cpp )
ament_target_dependencies(adsorbent1 std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(adsorbent1 ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(adsorbent2 src/ars_systems/adsorbent_tank_2.cpp )
ament_target_dependencies(adsorbent2 std_srvs rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(adsorbent2 ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ars src/ars_systems/ars_systems.cpp )
ament_target_dependencies(ars std_srvs rosidl_default_generators rclcpp sensor_msgs rclcpp_action diagnostic_msgs)
rosidl_target_interfaces(ars ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sabatier src/ors_systems/src/sabatier.cpp)
ament_target_dependencies(sabatier std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(sabatier ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(water_pub src/ors_systems/src/water_pub.cpp)
ament_target_dependencies(water_pub std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(water_pub ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(deionization_bed src/ors_systems/src/deionization_chamber.cpp)
ament_target_dependencies(deionization_bed std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(deionization_bed ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(ogs src/ors_systems/src/ogs_systems.cpp)
ament_target_dependencies(ogs std_srvs rosidl_default_generators rclcpp rclcpp_action diagnostic_msgs)

rosidl_target_interfaces(ogs ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(electrolysis src/ors_systems/src/electrolysis.cpp)
ament_target_dependencies(electrolysis std_srvs rosidl_default_generators rclcpp sensor_msgs)

rosidl_target_interfaces(electrolysis ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(whc src/wrs_systems/src/whc_controller.cpp)
ament_target_dependencies(whc rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(whc ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(waste_collector src/wrs_systems/src/waste_collection.cpp)
ament_target_dependencies(waste_collector rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(waste_collector ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(upa src/wrs_systems/src/urine_processor_assembly.cpp)
ament_target_dependencies(upa rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(upa ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(filter src/wrs_systems/src/water_processor_assembly/filteration_unit.cpp)
ament_target_dependencies(filter rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(filter ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(catalytic_chamber src/wrs_systems/src/water_processor_assembly/catalytic_chamber.cpp)
ament_target_dependencies(catalytic_chamber rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(catalytic_chamber ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ionizaton src/wrs_systems/src/water_processor_assembly/ionization_bed.cpp)
ament_target_dependencies(ionizaton rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(ionizaton ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(clean_water_tank src/wrs_systems/src/water_processor_assembly/product_water_tank.cpp)
ament_target_dependencies(clean_water_tank rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(clean_water_tank ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ultrasound src/wrs_systems/src/ultrasound_sensor.cpp)
ament_target_dependencies(ultrasound rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(ultrasound ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(crew_quarters src/crew_quarters.cpp)
ament_target_dependencies(crew_quarters rosidl_default_generators rclcpp rclcpp_action)
rosidl_target_interfaces(crew_quarters ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wrs src/wrs_systems/src/wrs_systems.cpp)
ament_target_dependencies(wrs rosidl_default_generators rclcpp rclcpp_action diagnostic_msgs std_srvs)
rosidl_target_interfaces(wrs ${PROJECT_NAME} "rosidl_typesupport_cpp")


# CDRA SAFETY


add_executable(safety src/stl_monitor.cpp)
target_sources(safety PRIVATE
  src/stl_monitor.cpp
)

ament_target_dependencies(safety rosidl_default_generators rclcpp sensor_msgs)
rosidl_target_interfaces(safety ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  ars
  ogs
  wrs
  crew_quarters
  safety
  collector
  desiccant1
  desiccant2
  adsorbent1 
  adsorbent2
  sabatier
  water_pub
  deionization_bed
  electrolysis
  whc
  waste_collector
  upa
  filter
  catalytic_chamber
  ionizaton
  clean_water_tank
  ultrasound
  safety
  
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/iss_simulator.py
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
