#include <geometry_msgs/msg/pose_stamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "moveit_control2",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_control2");

  // Setup the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface =
      MoveGroupInterface(node, "arm"); // Replace with your planning group

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = -0.303061;
  target_pose.orientation.y = -0.566473;
  target_pose.orientation.z = 0.624680;
  target_pose.orientation.w = 0.443889;
  target_pose.position.x = -0.018829;
  target_pose.position.y = -0.252053;
  target_pose.position.z = 0.578420;
  move_group_interface.setPoseTarget(
      target_pose, "R5A_link5"); // Replace with your end-effector link

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

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
