cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_eigen REQUIRED)



# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
)

# Add executables
add_executable(moveit_control_simple src/moveit_control_simple.cpp)
add_executable(end_pose_node src/end_pose_node.cpp)
add_executable(aruco_detector_single src/aruco_detector_single.cpp)
add_executable(moveit_control_gui src/moveit_control_gui.cpp)
add_executable(visual_joint_state_publisher src/visual_joint_state_publisher.cpp)
add_executable(aruco_rotation_publisher src/aruco_rotation_publisher.cpp)
add_executable(aruco_detector_double src/aruco_detector_double.cpp)

ament_target_dependencies(
  aruco_detector_double
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  cv_bridge
  OpenCV
  Eigen3
  yaml-cpp
  message_filters
)

target_link_libraries(aruco_detector_double
  yaml-cpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)


# Link libraries
ament_target_dependencies(aruco_rotation_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  aruco_rotation_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Link the required dependencies for moveit_control_gui
ament_target_dependencies(moveit_control_gui
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    tf2_ros
)

# Link Qt5Widgets using target_link_libraries
target_link_libraries(moveit_control_gui
  Qt5::Widgets
)

# Link libraries for aruco_detector_single (ArucoDetectorSingle)
ament_target_dependencies(
  aruco_detector_single
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

target_link_libraries(aruco_detector_single
  yaml-cpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Link libraries for moveit_control_simple
ament_target_dependencies(moveit_control_simple
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Link libraries for end_pose_node
ament_target_dependencies(end_pose_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries for visual_joint_state_publisher
ament_target_dependencies(visual_joint_state_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(visual_joint_state_publisher
  yaml-cpp
  Eigen3::Eigen
)

# Install targets
install(TARGETS
  aruco_detector_single
  moveit_control_simple
  end_pose_node
  moveit_control_gui
  visual_joint_state_publisher
  aruco_detector_double
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
