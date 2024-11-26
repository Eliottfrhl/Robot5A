#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * @brief A ROS2 node that moves the robot arm through a sequence of predefined joint configurations.
 *
 * This node is designed to perform a standard test by moving the robotic arm to different joint angles
 * configurations one after the other. The test is repeatable and can be used to collect consistent data
 * across multiple runs.
 */
int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("standard_joint_test_node");

  // Declare and get the joint positions parameter
  node->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>());
  std::vector<double> joint_positions_flat = node->get_parameter("joint_positions").as_double_array();

  // Check if the parameter is empty
  if (joint_positions_flat.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joint_positions' is empty or not set.");
    return 1;
  }

  // Assuming the robot has 5 joints
  const size_t num_joints = 5;
  if (joint_positions_flat.size() % num_joints != 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joint_positions' size is not a multiple of %zu.", num_joints);
    return 1;
  }

  // Convert the flat vector into a vector of joint configurations
  std::vector<std::vector<double>> joint_goals;
  for (size_t i = 0; i < joint_positions_flat.size(); i += num_joints)
  {
    std::vector<double> joint_values(joint_positions_flat.begin() + i, joint_positions_flat.begin() + i + num_joints);
    joint_goals.push_back(joint_values);
  }

  // Create the MoveGroupInterface for controlling the robot arm
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Iterate over the joint configurations
  for (size_t i = 0; i < joint_goals.size(); ++i)
  {
    // Set the joint target
    move_group.setJointValueTarget(joint_goals[i]);

    // Plan and execute the movement
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node->get_logger(), "Plan %zu successful! Executing...", i+1);
      move_group.execute(my_plan);
      RCLCPP_INFO(node->get_logger(), "Motion %zu executed.", i+1);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Planning for configuration %zu failed!", i+1);
    }

    // Wait for a short duration between movements
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
