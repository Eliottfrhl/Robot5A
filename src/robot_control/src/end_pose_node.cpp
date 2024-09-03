#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class EndPoseNode : public rclcpp::Node
{
public:
    EndPoseNode()
        : Node("end_pose_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EndPoseNode::publishEndEffectorPose, this));
    }

private:
    void publishEndEffectorPose()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("base_link", "R5A_link5", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to R5A_link5: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = transformStamped.header.stamp;
        pose.header.frame_id = transformStamped.header.frame_id;
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;

        publisher_->publish(pose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndPoseNode>());
    rclcpp::shutdown();
    return 0;
}
