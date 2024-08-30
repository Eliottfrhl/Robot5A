#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <iomanip>  // Include for setting precision

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("moveit_cpp_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Starting MoveItCpp demo...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  static const std::string PLANNING_GROUP = "arm"; // Update with your planning group name
  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);

  // Get the current position of the robotic arm
  auto current_state = moveit_cpp_ptr->getCurrentState();
  auto current_pose = current_state->getGlobalLinkTransform("R5A_link5");

  RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Current position: x = %.6f, y = %.6f, z = %.6f",
        current_pose.translation().x(),
        current_pose.translation().y(),
        current_pose.translation().z());

  // Setting up the goal position for the end effector with high precision
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";  // The base frame of your robot

  // Using high precision values
  target_pose.pose.position.x = std::round((current_pose.translation().x() - 0.1) * 1e6) / 1e6;
  target_pose.pose.position.y = std::round((current_pose.translation().y() - 0.1) * 1e6) / 1e6;
  target_pose.pose.position.z = std::round((current_pose.translation().z() - 0.1) * 1e6) / 1e6;

  // Convert the Eigen rotation to a geometry_msgs::msg::Quaternion
  target_pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(current_pose.rotation()));

  RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Goal position: x = %f, y = %f, z = %f",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);

  RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Goal orientation: x = %f, y = %f, z = %f, w = %f",
    target_pose.pose.orientation.x,
    target_pose.pose.orientation.y,
    target_pose.pose.orientation.z,
    target_pose.pose.orientation.w);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(target_pose, "R5A_link5");  // Set your end effector link name

  // Set goal tolerance
  //planning_components->setGoalPositionTolerance(0.01);  // 1 cm tolerance in position
  //planning_components->setGoalOrientationTolerance(0.1);  // ~5.7 degrees tolerance in orientation

  // Plan the trajectory
  auto plan_solution = planning_components->plan();
  if (plan_solution)
  {
    RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Planning successful!");

    // Execute the trajectory
    planning_components->execute();
    RCLCPP_INFO(rclcpp::get_logger("moveit_cpp_node"), "Execution complete!");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_cpp_node"), "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
