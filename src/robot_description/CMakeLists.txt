cmake_minimum_required(VERSION 3.8)
project(robot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)  # This will bring in joint_state_controller and others
find_package(robot_state_publisher REQUIRED)

# Install directories
install(DIRECTORY launch urdf meshes config worlds media
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/r5a_v_ros.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
