cmake_minimum_required(VERSION 3.8)
project(ur5_simple_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(ur5_simple_controller src/ur5_simple_controller.cpp)

ament_target_dependencies(ur5_simple_controller
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

# Install executable
install(TARGETS ur5_simple_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()