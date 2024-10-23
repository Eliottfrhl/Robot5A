/// @file moveit_control_simple.cpp
/// @brief Simple ROS2 node using MoveIt to move a robot arm to a target pose.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

/// @brief Main function that initializes the node and moves the robot arm to a
/// target pose.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char *argv[]) {
  // Initialize ROS 2 and create the node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "moveit_control2",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS 2 logger
  auto const logger = rclcpp::get_logger("moveit_control2");

  // Setup the MoveIt MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  /// Replace "arm" with your planning group name
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = -0.303061;
  target_pose.orientation.y = -0.566473;
  target_pose.orientation.z = 0.624680;
  target_pose.orientation.w = 0.443889;
  target_pose.position.x = -0.018829;
  target_pose.position.y = -0.252053;
  target_pose.position.z = 0.578420;

  /// Replace "R5A_link5" with your end-effector link name
  move_group_interface.setPoseTarget(target_pose, "R5A_link5");

  // Plan to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "Plan successful! Executing...");
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
