cmake_minimum_required(VERSION 3.5)
project(hyperdog_gazebo_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(hyperdog_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(hyperdog_gazebo_joint_ctrl_node src/hyperdog_gazebo_joint_controller.cpp)
ament_target_dependencies(hyperdog_gazebo_joint_ctrl_node
  rclcpp  
  std_msgs 
)

add_executable(fake_odom src/fake_odom.cpp)
ament_target_dependencies(fake_odom
  rclcpp  
  std_msgs 
  hyperdog_msgs
  nav_msgs
)

install(TARGETS
hyperdog_gazebo_joint_ctrl_node  
  DESTINATION lib/${PROJECT_NAME}
) 

install(TARGETS
fake_odom  
  DESTINATION lib/${PROJECT_NAME}
) 

install(DIRECTORY
  config
  description
  launch
  meshes
  worlds
  src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
