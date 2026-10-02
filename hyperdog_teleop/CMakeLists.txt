cmake_minimum_required(VERSION 3.16)
project(hyperdog_teleop LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hyperdog_msgs REQUIRED)

add_executable(hyperdog_teleop_joy_node src/hyperdog_teleop_joy.cpp)
ament_target_dependencies(hyperdog_teleop_joy_node rclcpp sensor_msgs geometry_msgs hyperdog_msgs)

install(TARGETS hyperdog_teleop_joy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
