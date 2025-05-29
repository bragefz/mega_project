cmake_minimum_required(VERSION 3.8)
project(moveit_wrapper)

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
add_executable(moveit_wrapper_node src/moveit_wrapper_node.cpp)

ament_target_dependencies(moveit_wrapper_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

# Install executable
install(TARGETS moveit_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()