#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class MoveitNode : public rclcpp::Node
{
public:
    MoveitNode()
    : Node("moveit_node")
    {
        // Initialize MoveGroupInterface for the "arm" group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

        // Set the target pose
        set_target_pose();

        // Plan and execute the motion
        plan_and_execute();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    void set_target_pose()
    {
        // Define the target position and orientation for link5
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;

        // Set orientation using a quaternion
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, 0); // roll, pitch, yaw
        target_pose.orientation.x = orientation.x();
        target_pose.orientation.y = orientation.y();
        target_pose.orientation.z = orientation.z();
        target_pose.orientation.w = orientation.w();

        // Set the target pose for link5
        move_group_->setPoseTarget(target_pose, "R5A_link5");
    }

    void plan_and_execute()
    {
        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
            // Execute the planned trajectory
            move_group_->move();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
