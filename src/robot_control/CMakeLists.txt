cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

# Include directories
include_directories(include)

# Add other executables (if any)
add_executable(moveit_control_simple src/moveit_control_simple.cpp)
add_executable(end_pose_node src/end_pose_node.cpp)
add_executable(object_detector src/object_detector.cpp)
add_executable(moveit_control_gui src/moveit_control_gui.cpp)

# Link the required dependencies
ament_target_dependencies(moveit_control_gui
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    tf2_ros
    Qt5Widgets  # Ensure Qt5Widgets is linked for GUI
)

# Link libraries for object detector
ament_target_dependencies(
  object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  yaml-cpp
  OpenCV
  Eigen3
)

target_link_libraries(object_detector yaml-cpp)  # Add this to explicitly link the yaml-cpp library

# Link libraries for other executables
ament_target_dependencies(moveit_control_simple
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(end_pose_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  object_detector
  moveit_control_simple
  end_pose_node
  moveit_control_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
