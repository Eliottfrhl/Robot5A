#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode() : Node("arm_control_node")
    {
        // Initialize MoveIt Interface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "arm");

        // Set up parameters
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);

        // Initialize Action Client for the controller
        action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "/arm_controller/follow_joint_trajectory");

        // Wait for the action server to be ready
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        }

        // Example target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.5;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.5;

        // Plan to the target pose
        planToTarget(target_pose);
    }

private:
    void planToTarget(const geometry_msgs::msg::Pose& target_pose)
    {
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan successful, executing...");
            executeTrajectory(my_plan.trajectory_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Plan failed");
        }
    }

    void executeTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory = trajectory.joint_trajectory;

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [](std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future)
        {
            auto goal_handle = future.get();
            if (!goal_handle)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.result_callback =
            [](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal successfully executed");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal failed with status: %d", result.code);
            }
        };

        // Send the goal to the action server
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
