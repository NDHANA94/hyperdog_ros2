cmake_minimum_required(VERSION 3.16)
project(hyperdog_bldc_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(DEPS
  controller_interface
  hardware_interface
  hyperdog_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
foreach(dep IN ITEMS ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bldc_impedance_controller.cpp
  src/mit_can_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPS})

# bring-up tools
add_executable(mit_can_probe src/tools/mit_can_probe.cpp)
target_include_directories(mit_can_probe PRIVATE include)
add_executable(joint_test src/tools/joint_test.cpp)
ament_target_dependencies(joint_test rclcpp hyperdog_msgs sensor_msgs)
install(TARGETS mit_can_probe joint_test DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(controller_interface bldc_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bldc_motor_model test/test_bldc_motor_model.cpp)
  target_include_directories(test_bldc_motor_model PRIVATE include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
