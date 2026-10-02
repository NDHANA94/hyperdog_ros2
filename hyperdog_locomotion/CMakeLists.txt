cmake_minimum_required(VERSION 3.16)
project(hyperdog_locomotion LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hyperdog_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# ---- ROS independent control library
add_library(${PROJECT_NAME}_core SHARED
  src/kinematics.cpp
  src/qp_solver.cpp
  src/gait_scheduler.cpp
  src/swing.cpp
  src/estimator.cpp
  src/balance.cpp
  src/locomotion_controller.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)

# ---- ROS 2 node
add_executable(locomotion_node src/locomotion_node.cpp)
target_link_libraries(locomotion_node ${PROJECT_NAME}_core)
ament_target_dependencies(locomotion_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros hyperdog_msgs ros_gz_interfaces)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS locomotion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_locomotion_core
    test/test_kinematics.cpp
    test/test_qp_balance.cpp
    test/test_gait_estimator.cpp)
  target_link_libraries(test_locomotion_core ${PROJECT_NAME}_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
