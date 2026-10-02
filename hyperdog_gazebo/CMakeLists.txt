cmake_minimum_required(VERSION 3.16)
project(hyperdog_gazebo LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hyperdog_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(scenario_runner
  src/scenario_runner.cpp
  src/scenarios.cpp
)
ament_target_dependencies(scenario_runner rclcpp geometry_msgs nav_msgs hyperdog_msgs ros_gz_interfaces)

install(TARGETS scenario_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
