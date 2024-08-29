cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(moveit_node src/moveit_node.cpp)

# Link libraries
ament_target_dependencies(moveit_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)

# Install targets
install(TARGETS
  moveit_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
