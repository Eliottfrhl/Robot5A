cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)


# Include directories
include_directories(include)

# Add executable
add_executable(moveit_control src/moveit_control.cpp)
add_executable(moveit_control2 src/moveit_control2.cpp)

# Link libraries
ament_target_dependencies(
  moveit_control
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)

# Link libraries
ament_target_dependencies(moveit_control2
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Install targets
install(TARGETS
  moveit_control
  moveit_control2
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
